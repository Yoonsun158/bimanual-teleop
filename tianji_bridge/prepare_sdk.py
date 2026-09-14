#!/usr/bin/env python3
"""Fetch immutable vendor sources, verify Git blob hashes, apply integration patches."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request

COMMIT = "02440e886fb59095711eb9ec6dcbedd8be08922a"
REPOSITORY = "cynthia-you/TJ_FX_ROBOT_CONTRL_SDK"
MODIFICATION_NOTICE = "// Modified by bimanual_teleop; see tianji_bridge/README.md for patch details.\n"
def download(url):
    request = urllib.request.Request(url, headers={"User-Agent": "bimanual-teleop-build"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def prepare(destination):
    original = destination / "sdk" / COMMIT
    original.mkdir(parents=True, exist_ok=True)
    manifest_path = original / "manifest.json"
    previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    entries = {name: digest for name, digest in previous.items()
               if not name.startswith("CommonConfig/LoadData_ccs/")}
    if "LICENSE" not in entries:
        tree = json.loads(download(f"https://api.github.com/repos/{REPOSITORY}/git/trees/{COMMIT}?recursive=1"))
        entries = {item["path"]: item["sha"] for item in tree["tree"]
                   if item["type"] == "blob" and (
                       (item["path"].split("/")[0] in {"contrlSDK100343", "kinematicsSDK"}
                        and Path(item["path"]).suffix in {".h", ".cpp"})
                       or item["path"] in {"LICENSE", "CommonConfig/ccs_m6_40.MvKDCfg", "robot.ini"})}
    if entries != previous:
        manifest_path.write_text(json.dumps(entries, indent=2) + "\n")

    def fetch(item):
        name, expected = item
        path = original / name
        data = path.read_bytes() if path.exists() else download(
            f"https://raw.githubusercontent.com/{REPOSITORY}/{COMMIT}/{name}")
        actual = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        if actual != expected:
            raise ValueError(f"Git blob hash mismatch: {name}")
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        return name, data

    with ThreadPoolExecutor(max_workers=6) as pool:
        for name, data in pool.map(fetch, entries.items()):
            if name in {"contrlSDK100343/Robot.cpp", "contrlSDK100343/TCPAgent.cpp",
                        "contrlSDK100343/TCPAgent.h", "contrlSDK100343/TCPFileClient.cpp",
                        "contrlSDK100343/FileOP.cpp", "contrlSDK100343/FileOP.h"}:
                continue
            path = destination / "vendor" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists() or path.read_bytes() != data:
                path.write_bytes(data)

    path = destination / "vendor/contrlSDK100343/Robot.cpp"
    source = (original / "contrlSDK100343/Robot.cpp").read_text()

    def replace_once(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError(f"SDK hook anchor changed: {old[:80]!r}")
        source = source.replace(old, new)

    replace_once('#include "Robot.h"', '#include "Robot.h"\n#include "sdk_hooks.h"')
    # These routine messages ignore the SDK logging switches. Keep SDK error
    # output and version queries intact; remove only their unconditional prints.
    replace_once('\t\tprintf("system version:%ld\\n",ctrlSysVers);\n', "")
    replace_once('\tprintf("[Marvin SDK]: Robot released\\n");\n', "")
    # A powered controller may keep streaming even without a client. Isolate
    # loopback tests from that traffic; real connections retain the SDK binding.
    replace_once("m_InsRobot->_local.sin_addr.s_addr = INADDR_ANY;",
                 "m_InsRobot->_local.sin_addr.s_addr = ip1 == 127 ? htonl(INADDR_LOOPBACK) : INADDR_ANY;")
    # Timestamp the original datagram at recvfrom return, not a later latest read.
    receive = "Len = recvfrom(_local_sock, recvbuf, 2000, 0, (struct sockaddr *)&_local, (socklen_t *)&_localLen);"
    if source.count(receive) != 2:
        raise ValueError("SDK receive anchors changed")
    source = source.replace(receive, receive + "\n\t\ttj_hook_received(recvbuf, Len);")
    replace_once("void CRobot::DoSend()\n{", "void CRobot::DoSend()\n{\n\tstd::lock_guard<std::recursive_mutex> tj_lock(tj_sdk_mutex);")
    for function in ("OnClearSet", "OnSetSend"):
        replace_once(f"bool CRobot::{function}()\n{{",
                     f"bool CRobot::{function}()\n{{\n\tstd::lock_guard<std::recursive_mutex> tj_lock(tj_sdk_mutex);")
    # Bind the caller's token before the timer can observe a pending datagram.
    replace_once("\tm_InsRobot->m_SendTag = 100;\n\n\treturn true;\n}",
                 "\ttj_hook_publish();\n\tm_InsRobot->m_SendTag = 100;\n\n\treturn true;\n}")
    replace_once("sendto(_tosock_, (char *)m_SendBuf_, m_Slen, 0, (struct sockaddr *)&_to, sizeof(_to));",
                 "tj_hook_send(_tosock_, m_SendBuf_, m_Slen, (struct sockaddr *)&_to, sizeof(_to));")
    source = MODIFICATION_NOTICE + source
    if not path.exists() or path.read_text() != source:
        path.write_text(source)

    def patch_file(name, transform):
        path = destination / "vendor/contrlSDK100343" / name
        text = MODIFICATION_NOTICE + transform((original / "contrlSDK100343" / name).read_text())
        if not path.exists() or path.read_text() != text:
            path.write_text(text)

    def once(text, old, new):
        if text.count(old) != 1:
            raise ValueError(f"SDK download anchor changed: {old[:80]!r}")
        return text.replace(old, new)

    # A downloaded file must not outlive the thread writing it. The vendor's
    # detached TCP reader only slept 10 ms before destroying its receiving object.
    def tcp_header(text):
        text = once(text, '#include "CmplOpt.h"', '#include "CmplOpt.h"\n#include <thread>\n#include <atomic>')
        text = once(text, "class CTCPAgent  \n{", "class CTCPAgent  \n{\n    std::thread tj_receiver;")
        for old, new in (("bool m_bLinkTag;", "std::atomic<bool> m_bLinkTag;"),
                         ("SOCKET m_iSocket;", "std::atomic<SOCKET> m_iSocket;"),
                         ("bool m_quit_tag;", "std::atomic<bool> m_quit_tag;")):
            text = once(text, old, new)
        return text
    patch_file("TCPAgent.h", tcp_header)

    def tcp_agent(text):
        begin = text.index("#ifdef CMPL_LIN", text.index("bool CTCPAgent::OnQuit()"))
        end = text.index("#endif", begin) + len("#endif")
        text = text[:begin] + """#ifdef CMPL_LIN
    const int socket_to_close = m_iSocket;
    if (socket_to_close != INVALID_SOCKET) shutdown(socket_to_close, SHUT_RDWR);
    if (tj_receiver.joinable()) tj_receiver.join();
    if (socket_to_close != INVALID_SOCKET) close(socket_to_close);
    m_iSocket = INVALID_SOCKET;
    m_bLinkTag = false;
#endif""" + text[end:]
        begin = text.index("#ifdef CMPL_LIN", text.index("bool CTCPAgent::OnLinkTo("))
        end = text.index("#endif", begin) + len("#endif")
        text = text[:begin] + """#ifdef CMPL_LIN
    tj_receiver = std::thread([this] { NetLoop(this); });
#endif""" + text[end:]
        text = once(text, "\t//向服务器发出连接请求", """#ifdef CMPL_LIN
    timeval tj_timeout{2, 0};
    setsockopt(m_iSocket, SOL_SOCKET, SO_SNDTIMEO, &tj_timeout, sizeof(tj_timeout));
#endif
\t//向服务器发出连接请求""")
        text = once(text, '        printf("OnLinkTo ERR3\\n");',
                    '        printf("OnLinkTo ERR3\\n");\n#ifdef CMPL_LIN\n        close(m_iSocket);\n#endif')
        text = once(text, "\t\t\tsunit->m_iSocket = INVALID_SOCKET;", "")
        return once(text, "return (send(m_iSocket, s, slen, 0) != SOCKET_ERROR);", """while (slen > 0) {
        const auto written = send(m_iSocket, s, slen, MSG_NOSIGNAL);
        if (written <= 0) return false;
        s += written; slen -= written;
    }
    return true;""")

    patch_file("TCPAgent.cpp", tcp_agent)

    def file_client(text):
        # File transfers must terminate if the peer stops replying.
        anchor = "\twhile(m_fop.OnCheckStateOK() == false)"
        if text.count(anchor) != 2 or text.count("\t\tUninetSleep(50);") != 2:
            raise ValueError("SDK file transfer wait anchors changed")
        return text.replace(anchor, "\tfor (int remaining = 200; !m_fop.OnCheckStateOK(); --remaining)").replace(
            "\t\tUninetSleep(50);", "\t\tif (remaining == 0) return false;\n\t\tUninetSleep(50);")

    patch_file("TCPFileClient.cpp", file_client)
    def file_header(text):
        for old, new in (('#include "stdio.h"', '#include "stdio.h"\n#include <atomic>'),
                         ("File_OP_State m_state;", "std::atomic<File_OP_State> m_state;"),
                         ("bool m_ErrorTag;", "std::atomic<bool> m_ErrorTag;")):
            text = once(text, old, new)
        return text
    patch_file("FileOP.h", file_header)

    def file_io(text):
        begin = text.index("bool CFileOp::OnStateRecvCln(")
        end = text.index("\n}", begin) + 2
        part = text[begin:end]
        part = once(part, "\t\t\twltnum -= wn;", """            if (wn <= 0) {
                m_ErrorTag = true;
                fclose(m_fp); m_fp = NULL; m_state = File_OP_OK;
                return false;
            }
\t\t\twltnum -= wn;""")
        part = part.replace("fclose(m_fp);", "if (fclose(m_fp) != 0) m_ErrorTag = true;")
        return text[:begin] + part + text[end:]

    patch_file("FileOP.cpp", file_io)
    return original


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    print(prepare(args.destination.resolve()))
