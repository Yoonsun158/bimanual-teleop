"""Local Unix-socket CLI. Default backend is simulated; hardware is explicit."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import socketserver
import sys
import threading
import time
import uuid

from .camera import CameraCapture
from .runtime import Runner

DEFAULT_RUN = Path(__file__).parent / "runs" / "session"
MAX_REQUEST = 65536


def socket_path(run_dir):
    key = hashlib.sha256(str(Path(run_dir).resolve()).encode()).hexdigest()[:20]
    directory = Path("/tmp") / (f"astra-rail-{os.getuid()}-" + key)
    directory.mkdir(mode=0o700, exist_ok=True)
    if directory.is_symlink() or directory.stat().st_uid != os.getuid():
        raise RuntimeError("unsafe socket directory")
    directory.chmod(0o700)
    return directory / "control.sock"


def decode(data):
    def invalid(value):
        raise ValueError("nonfinite JSON value: " + value)
    result = json.loads(data, parse_constant=invalid)
    if not isinstance(result,dict):
        raise ValueError("request must be a JSON object")
    return result


def rpc(path, request):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(30.)
        client.connect(str(path))
        client.sendall(json.dumps(request,allow_nan=False).encode()+b"\n")
        with client.makefile("rb") as stream:
            response = decode(stream.readline(2_000_000))
        if not response.get("ok"):
            raise RuntimeError(response.get("error","request failed"))
        return response["result"]


def serve(args):
    run_dir = args.run_dir.resolve()
    path = socket_path(run_dir)
    if path.exists():
        try:
            rpc(path,{"op":"status"})
        except (ConnectionRefusedError,FileNotFoundError):
            path.unlink(missing_ok=True)
        else:
            raise RuntimeError("a runner already owns this run directory/socket")
    sides = ("left","right") if args.side == "both" else (args.side,)
    if args.backend == "hardware":
        from .hardware import HardwareBackend
        backend = HardwareBackend(sides=sides)
    else:
        from .fake import FakeBackend
        backend = FakeBackend(sides=sides)
    camera = CameraCapture(run_dir / "images", fake=args.backend == "fake")
    runner = Runner(backend,camera,run_dir,allow_motion=args.allow_motion)
    runner.start()

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(30)
            try:
                line = self.rfile.readline(MAX_REQUEST+1)
                if len(line) > MAX_REQUEST or not line.endswith(b"\n"):
                    raise ValueError("request too large or incomplete")
                result = runner.handle(decode(line))
                output = {"ok":True,"result":result}
            except Exception as error:
                output = {"ok":False,"error":str(error)}
            try:
                self.wfile.write(json.dumps(output,ensure_ascii=False,allow_nan=False).encode()+b"\n")
            except (BrokenPipeError,ConnectionResetError):
                pass  # Client lifetime is independent of device holding.

    class Server(socketserver.ThreadingUnixStreamServer):
        daemon_threads = True

    server = None
    def stop(signum, frame):
        try:
            if runner.state == "running":
                runner.pause(f"signal {signum}")
            runner.shutdown()
        except Exception as error:
            print(f"Cannot close runner: {error}; onsite intervention required",file=sys.stderr,flush=True)

    try:
        server = Server(str(path),Handler)
        os.chmod(path,0o600)
        server.timeout = .2
        signal.signal(signal.SIGTERM,stop)
        signal.signal(signal.SIGINT,stop)
        info = {"pid":os.getpid(),"session_id":runner.session_id,"socket":str(path),
                "run_dir":str(run_dir),"backend":args.backend,"sides":sides,
                "allow_motion":args.allow_motion}
        (run_dir / "session.json").write_text(json.dumps(info,indent=2))
        print(json.dumps(info),flush=True)
        while runner.state != "closed":
            server.handle_request()
    finally:
        if server:
            server.server_close()
        path.unlink(missing_ok=True)
        if runner.state != "closed":
            if runner.possible_load:
                runner.fault("server terminated unexpectedly while possibly loaded")
                # Existing driver shutdown is a fault path, never a promise of holding.
                backend.close()
                camera.close()
            else:
                if runner.state == "running":
                    runner.pause("server exiting")
                runner.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir",type=Path,default=DEFAULT_RUN)
    commands = parser.add_subparsers(dest="command",required=True)
    server = commands.add_parser("serve")
    server.add_argument("--backend",choices=("fake","hardware"),default="fake")
    server.add_argument("--side",choices=("both","left","right"),default="both")
    server.add_argument("--allow-motion",action="store_true")
    client = commands.add_parser("call")
    client.add_argument("op",choices=("status","observe","engage","arm-step","hand-step",
                                     "lift","mark","pause","resume","shutdown"))
    source = client.add_mutually_exclusive_group()
    source.add_argument("--args",default="{}")
    source.add_argument("--args-file",type=Path)
    client.add_argument("--request-id")
    args = parser.parse_args(argv)
    try:
        if args.command == "serve":
            serve(args)
        else:
            path = socket_path(args.run_dir)
            payload = decode(args.args_file.read_text() if args.args_file else args.args)
            request = {"op":args.op,"args":payload}
            if args.op not in ("status","observe"):
                current = rpc(path,{"op":"status"})
                request.update(session_id=current["session_id"],id=args.request_id or uuid.uuid4().hex,
                               expires_ns=time.monotonic_ns()+10_000_000_000)
            result = rpc(path,request)
            print(json.dumps(result,ensure_ascii=False,allow_nan=False,indent=2))
        return 0
    except Exception as error:
        print(f"astra-rail: {error}",file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
