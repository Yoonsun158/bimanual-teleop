/* Hardware-free integration test: only an explicitly bound loopback peer.
 * Exercises the patched vendor receive/send path and the public bridge ABI. */
#include "bridge.h"
#include "sdk_hooks.h"
#include "Robot.h"
#include "FileOP.h"
#include "Parser.h"
#include <arpa/inet.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (0)
using namespace std::chrono_literals;

class LoopbackController {
    int fd = -1;
    std::atomic<bool> running{true};
    std::mutex mutex;
    std::thread thread;
    DCSS state{};
    sockaddr_in destination{};
    void emit_locked() {
        ++state.m_Out[0].m_OutFrameSerial;
        ++state.m_Out[1].m_OutFrameSerial;
        unsigned char bytes[sizeof(DCSS) + 2] = {'F', 'X'};
        std::memcpy(bytes + 2, &state, sizeof state);
        sendto(fd, bytes, sizeof bytes, 0, reinterpret_cast<sockaddr *>(&destination), sizeof destination);
    }
    void serve() {
        unsigned char bytes[1500];
        while (running) {
            const int size = recv(fd, bytes, sizeof bytes, 0);
            if (size < 7 || bytes[0] != 'F' || bytes[1] != 'Y') continue;
            std::lock_guard<std::mutex> lock(mutex);
            for (int pos = 7; pos + 5 <= size;) {
                const int count = bytes[pos + 1] * 256 + bytes[pos + 2];
                if (pos + 5 + count > size) break;
                const auto *payload = bytes + pos + 5;
                if (bytes[pos] == 150 && count == 42) {
                    std::memcpy(state.m_ParaName, payload, 30);
                    std::memcpy(&state.m_ParaRetSerial, payload + 40, 2);
                    if (std::strncmp(state.m_ParaName, "RSTA", 4) == 0)
                        state.m_ParaRetSerial += 100 * stop_return_code.load();
                    if (std::strcmp(state.m_ParaName, "RESET0") == 0 || std::strcmp(state.m_ParaName, "RESET1") == 0) {
                        const int arm = state.m_ParaName[5] - '0';
                        state.m_ParaRetSerial += 100 * reset_return_code.load();
                        if (reset_return_code.load() == 0) {
                            state.m_State[arm].m_CurState = 0;
                            state.m_State[arm].m_ERRCode = 0;
                        }
                    }
                    state.m_ParaValueI = SDK_VERSION;
                    emit_locked();
                }
                pos += 5 + count;
            }
        }
    }
public:
    std::atomic<int> stop_return_code{0};
    std::atomic<int> reset_return_code{0};
    void set_emergency(int arm, float reported_velocity = 0) {
        std::lock_guard<std::mutex> lock(mutex);
        state.m_State[arm].m_CurState = 100;
        state.m_State[arm].m_ERRCode = 13;
        state.m_Out[arm].m_LowSpdFlag = 1;
        state.m_Out[arm].m_FB_Joint_Vel[0] = reported_velocity;
        emit_locked();
    }
    void set_mode(int arm, int mode, int impedance, int low_speed) {
        std::lock_guard<std::mutex> lock(mutex);
        state.m_State[arm].m_CurState = mode;
        state.m_In[arm].m_ImpType = impedance;
        state.m_Out[arm].m_LowSpdFlag = low_speed;
        emit_locked();
    }
    LoopbackController() {
        fd = socket(AF_INET, SOCK_DGRAM, 0);
        CHECK(fd >= 0);
        sockaddr_in address{}; address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(4729);
        CHECK(bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0);
        timeval timeout{0, 10000};
        CHECK(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout) == 0);
        destination = address; destination.sin_port = htons(4730);
        thread = std::thread([this] { serve(); });
    }
    ~LoopbackController() { running = false; if (thread.joinable()) thread.join(); if (fd >= 0) close(fd); }
    void burst(int count) {
        std::lock_guard<std::mutex> lock(mutex);
        for (int i = 0; i < count; ++i) {
            state.m_Out[0].m_FB_Joint_Pos[0] = float(i);
            state.m_Out[1].m_FB_Joint_Pos[0] = float(100 + i);
            state.m_Out[0].m_FB_Joint_Cmd[0] = 42;
            state.m_In[0].m_Joint_CMD_Pos[0] = -42;
            emit_locked();
        }
    }
};

class LoopbackFileServer {
    int listener = -1;
    std::thread thread;
public:
    explicit LoopbackFileServer(bool partial) {
        listener = socket(AF_INET, SOCK_STREAM, 0);
        CHECK(listener >= 0);
        int reuse = 1; setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse);
        sockaddr_in address{}; address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(10240);
        CHECK(bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof address) == 0);
        CHECK(listen(listener, 1) == 0);
        thread = std::thread([this, partial] {
            const int client = accept(listener, nullptr, nullptr);
            if (client < 0) return;
            CParser parser; char bytes[32768]; bool done = false;
            while (!done) {
                const auto size = recv(client, bytes, sizeof bytes, 0);
                if (size <= 0) break;
                parser.OnAddRawData(bytes, size);
                while (parser.OnUnPack()) {
                    long size = 0; const char *payload = parser.OnGetContent(size);
                    if (size != sizeof(FileIns)) continue;
                    FileIns request{}; std::memcpy(&request, payload, sizeof request);
                    FileIns response{}; response.m_TotalBlockNum = partial ? 2 : 1;
                    if (request.m_InsType == INS_TYPE_Get_Request) {
                        response.m_InsType = INS_TYPE_Get_Request_Report;
                    } else if (request.m_InsType == INS_TYPE_Get_File_Cell && request.m_CurrentBlockSerial == 0) {
                        response.m_InsType = INS_TYPE_Get_File_Cell_Report;
                        const char text[] = "[R.A0.BASIC]\nType=1017\n[R.A1.BASIC]\nType=1017\n";
                        response.m_CellSize = sizeof(text) - 1;
                        std::memcpy(response.m_CellContent, text, sizeof(text) - 1);
                        done = partial;
                    } else continue;
                    long count = 0;
                    const char *packed = parser.OnPack(reinterpret_cast<char *>(&response), sizeof response, count);
                    while (count > 0) {
                        const auto written = send(client, packed, count, MSG_NOSIGNAL);
                        if (written <= 0) { done = true; break; }
                        packed += written; count -= written;
                    }
                }
            }
            shutdown(client, SHUT_RDWR); close(client);
        });
    }
    ~LoopbackFileServer() {
        shutdown(listener, SHUT_RDWR); close(listener);
        if (thread.joinable()) thread.join();
    }
};

TjSend wait_send(uint64_t token) {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    do {
        TjSend value{};
        while (tj_poll_send(&value) == 1) if (value.token == token) return value;
        std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("missing send receipt");
}
std::vector<int> instructions(const TjSend &value) {
    std::vector<int> result;
    CHECK(value.size >= 7);
    for (unsigned pos = 7; pos + 5 <= value.size;) {
        result.push_back(value.payload[pos]);
        pos += 5 + value.payload[pos + 1] * 256 + value.payload[pos + 2];
        CHECK(pos <= value.size);
    }
    return result;
}

void benchmark_submit(const double q[14], uint64_t first_token) {
    constexpr uint64_t period_ns = 5000000;
    const uint64_t start = tj_now_ns() + period_ns;
    std::vector<uint64_t> durations, send_times;
    int skipped = 0, rejected = 0, accepted = 0, expired = 0, failures = 0;
    uint64_t token = first_token;
    for (int cycle = 0; cycle < 100;) {
        const uint64_t deadline = start + cycle * period_ns;
        std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(deadline)));
        const uint64_t now = tj_now_ns();
        if (now >= deadline + period_ns) {
            const int count = std::min(100 - cycle, int((now - deadline) / period_ns));
            skipped += count; cycle += count; continue;
        }
        const int result = tj_submit(3, q, token++, deadline + period_ns);
        durations.push_back(tj_now_ns() - now);
        if (result == 0) ++accepted; else ++rejected;
        ++cycle;
    }
    std::this_thread::sleep_for(10ms);
    TjSend receipt{};
    while (tj_poll_send(&receipt) == 1) if (receipt.token >= first_token) {
        if (!receipt.attempted) ++expired;
        else if (receipt.result != receipt.size) ++failures;
        else send_times.push_back(receipt.sent_ns);
    }
    CHECK(int(send_times.size()) + expired + failures == accepted);
    std::sort(durations.begin(), durations.end());
    uint64_t min_interval = 0, max_interval = 0;
    for (size_t i = 1; i < send_times.size(); ++i) {
        const auto interval = send_times[i] - send_times[i - 1];
        if (i == 1 || interval < min_interval) min_interval = interval;
        max_interval = std::max(max_interval, interval);
    }
    const auto percentile = [&](double p) { return durations.empty() ? 0 : durations[size_t((durations.size() - 1) * p)]; };
    std::cout << "{\"benchmark\":\"loopback_submit_200_hz\",\"planned_cycles\":100"
              << ",\"accepted\":" << accepted << ",\"rejected\":" << rejected
              << ",\"skipped_cycles\":" << skipped << ",\"expired_before_send\":" << expired
              << ",\"send_failures\":" << failures << ",\"successful_sends\":" << send_times.size()
              << ",\"submit_p50_ns\":" << percentile(0.5) << ",\"submit_p95_ns\":" << percentile(0.95)
              << ",\"submit_max_ns\":" << percentile(1) << ",\"send_interval_min_ns\":" << min_interval
              << ",\"send_interval_max_ns\":" << max_interval << "}\n";
}

int main() {
    try {
        LoopbackController controller;
        CHECK(tj_abi_version() == 2);
        CHECK(tj_open("127.0.0.1") == 0);
        CHECK(tj_open("127.0.0.1") == -2);
        int64_t sdk = 0, version = 0;
        CHECK(tj_versions(&sdk, &version) == 0 && sdk == SDK_VERSION && version == sdk);
        TjFeedback value{};
        while (tj_poll_feedback(&value) == 1) {}
        controller.burst(10);
        std::vector<TjFeedback> frames;
        for (int i = 0; i < 100 && frames.size() < 10; ++i) {
            // VERSION retries can emit unrelated frames around the burst.
            while (tj_poll_feedback(&value) == 1)
                if (value.target[0] == 42 && value.q[0] == double(frames.size()))
                    frames.push_back(value);
            std::this_thread::sleep_for(1ms);
        }
        CHECK(frames.size() == 10);
        for (size_t i = 0; i < frames.size(); ++i) {
            CHECK(frames[i].q[0] == double(i) && frames[i].q[7] == double(100 + i));
            CHECK(frames[i].target[0] == 42);
            if (i) {
                CHECK(frames[i].packet_index == frames[i - 1].packet_index + 1);
                CHECK(frames[i].received_ns >= frames[i - 1].received_ns);
            }
        }
        TjFeedback latest{}; CHECK(tj_latest(&latest) == 1);
        CHECK(tj_latest(&value) == 1 && value.received_ns == latest.received_ns);

        double q[14]{};
        // Prevent the sender from draining the slot: a new target must time out
        // without overwriting the old packet or consuming its own token.
        {
            std::lock_guard<std::recursive_mutex> lock(tj_sdk_mutex);
            CHECK(tj_submit(3, q, 1, tj_now_ns() + 100000000ULL) == 0);
            const uint64_t started = tj_now_ns();
            CHECK(tj_submit(3, q, 2, tj_now_ns() + 100000000ULL) == -2);
            CHECK(tj_now_ns() - started >= 5000000ULL);
            CHECK(tj_now_ns() - started < 50000000ULL);
            CHECK(tj_submit(3, q, 2, tj_now_ns() + 1000000ULL) == -3);
            CHECK(std::strstr(tj_last_error(), "expired waiting") != nullptr);
        }
        auto outcome = wait_send(1);
        CHECK(outcome.attempted == 1 && outcome.result == outcome.size && outcome.error_number == 0);
        CHECK(instructions(outcome) == std::vector<int>({108, 208}));
        {
            std::lock_guard<std::recursive_mutex> lock(tj_sdk_mutex);
            CHECK(tj_submit(1, q, 2, tj_now_ns() + 1000000ULL) == 0);
            std::this_thread::sleep_for(3ms);
        }
        outcome = wait_send(2);
        CHECK(outcome.attempted == 0 && outcome.result == -1 && outcome.error_number == ETIMEDOUT);

        TjProfile profiles[2]{};
        for (auto &profile : profiles) {
            profile.velocity_ratio = profile.acceleration_ratio = 1;
            profile.tool_dynamics[0] = 1;
            for (int i = 0; i < 7; ++i) { profile.k[i] = 1; profile.d[i] = 0.1; }
        }
        CHECK(tj_configure(3, profiles, 3) == 0);
        outcome = wait_send(3);
        CHECK(instructions(outcome) == std::vector<int>({102, 103, 105, 107, 121, 202, 203, 205, 207, 221}));
        CHECK(tj_engage(1, q, 4, tj_now_ns() + 100000000ULL) == 0);
        outcome = wait_send(4);
        CHECK(instructions(outcome) == std::vector<int>({108, 111, 101}));
        CHECK(tj_hold(1, 5) == 0);
        outcome = wait_send(5);
        CHECK(instructions(outcome) == std::vector<int>({150}));
        CHECK(std::strcmp(reinterpret_cast<const char *>(outcome.payload + 12), "RSTA0") == 0);
        benchmark_submit(q, 6);

        // Back-to-back submissions must wait for the SDK timer rather than
        // reject a healthy asynchronous send. Every accepted token is sent once.
        for (uint64_t token = 120; token < 152; ++token)
            CHECK(tj_submit(3, q, token, tj_now_ns() + 100000000ULL) == 0);
        uint64_t packet_index = 0;
        for (uint64_t token = 120; token < 152; ++token) {
            outcome = wait_send(token);
            CHECK(outcome.attempted == 1 && outcome.result == outcome.size);
            CHECK(instructions(outcome) == std::vector<int>({108, 208}));
            CHECK(token == 120 || outcome.packet_index == packet_index + 1);
            packet_index = outcome.packet_index;
        }
        CHECK(tj_submit(3, q, 151, tj_now_ns() + 100000000ULL) == -3);
        CHECK(std::strstr(tj_last_error(), "token must increase") != nullptr);

        for (int arm = 0; arm < 2; ++arm) {
            const uint64_t token = 220 + arm * 2;
            double measured[7] = {1, 2, 3, 4, 5, 6, 7};
            CHECK(tj_position_mode(arm, measured, token, tj_now_ns() + 100000000ULL) == 0);
            outcome = wait_send(token);
            CHECK(instructions(outcome) == std::vector<int>({103 + 100 * arm, 108 + 100 * arm, 101 + 100 * arm}));
            // Velocity/acceleration percentages and seed are in the same datagram.
            int16_t ratios[2]; std::memcpy(ratios, outcome.payload + 12, sizeof ratios);
            CHECK(ratios[0] == 100 && ratios[1] == 100);
            CHECK(tj_hold(1u << arm, token + 1) == 0);
            outcome = wait_send(token + 1);
            CHECK(std::strcmp(reinterpret_cast<const char *>(outcome.payload + 12), arm ? "RSTA1" : "RSTA0") == 0);
        }
        CHECK(tj_position_mode(2, q, 224, tj_now_ns() + 100000000ULL) == -3);
        CHECK(tj_position_mode(0, nullptr, 224, tj_now_ns() + 100000000ULL) == -3);
        CHECK(tj_position_mode(0, q, 224, tj_now_ns() - 1) == -3);

        // Retire a rejected position stop only after real Cartesian feedback.
        CHECK(tj_position_mode(0, q, 224, tj_now_ns() + 100000000ULL) == 0);
        wait_send(224);
        CHECK(tj_engage(1, q, 225, tj_now_ns() + 100000000ULL) == 0);
        wait_send(225);
        CHECK(tj_confirm_cartesian(1) == -1);
        controller.stop_return_code = 1;
        CHECK(tj_hold(1, 226) == -1);
        wait_send(226);
        controller.set_mode(0, 3, 2, 1);
        int confirmed = -1;
        for (int attempt = 0; attempt < 100 && confirmed != 0; ++attempt) {
            std::this_thread::sleep_for(1ms);
            confirmed = tj_confirm_cartesian(1);
        }
        CHECK(confirmed == 0);
        // The standard Cartesian stop may return without a datagram at low speed.
        CHECK(tj_hold(1, 227) == 0);
        controller.stop_return_code = 0;

        char path[] = "/tmp/tianji-native-XXXXXX";
        CHECK(tj_reset_emergency(0, 228, tj_now_ns()+100000000ULL) == -3);
        for (int arm = 0; arm < 2; ++arm) {
            // Reset uses the controller's stop flag, not a second host speed threshold.
            controller.set_emergency(arm, 1.0f);
            TjFeedback observed{};
            for (int attempt = 0; attempt < 100; ++attempt) {
                std::this_thread::sleep_for(1ms);
                if (tj_latest(&observed) == 1 && observed.error[arm] == 13) break;
            }
            CHECK(observed.error[arm] == 13);
            const uint64_t token = 230+arm*2;
            CHECK(tj_reset_emergency(arm, token, tj_now_ns()-1) == -3);
            controller.reset_return_code = 1;
            CHECK(tj_reset_emergency(arm, token, tj_now_ns()+100000000ULL) == -1);
            outcome = wait_send(token);
            CHECK(instructions(outcome) == std::vector<int>({150}));
            CHECK(std::strcmp(reinterpret_cast<const char *>(outcome.payload+12), arm ? "RESET1" : "RESET0") == 0);
            controller.reset_return_code = 0;
            CHECK(tj_reset_emergency(arm, token+1, tj_now_ns()+100000000ULL) == 0);
            wait_send(token+1);
        }
        for (int arm = 0; arm < 2; ++arm) {
            const uint64_t token = 240 + arm * 2;
            const double target[7] = {arm ? -35.0 : 35.0, -55, 0, -65, 0, 0, 0};
            CHECK(tj_move_joints(arm, target, 65, 40, token, tj_now_ns()+100000000ULL) == 0);
            outcome = wait_send(token);
            CHECK(outcome.attempted == 1 && outcome.result == outcome.size);
            CHECK(instructions(outcome) == std::vector<int>({103 + 100 * arm, 108 + 100 * arm, 101 + 100 * arm}));
            int16_t ratios[2]; std::memcpy(ratios, outcome.payload + 12, sizeof ratios);
            CHECK(ratios[0] == 65 && ratios[1] == 40);
            float joints[7]; std::memcpy(joints, outcome.payload + 25, sizeof joints);
            for (int joint = 0; joint < 7; ++joint) CHECK(joints[joint] == float(target[joint]));
            int32_t mode; std::memcpy(&mode, outcome.payload + 58, sizeof mode);
            CHECK(mode == 1);
            CHECK(tj_hold(1u << arm, token + 1) == 0);
            outcome = wait_send(token + 1);
            CHECK(std::strcmp(reinterpret_cast<const char *>(outcome.payload+12), arm ? "RSTA1" : "RSTA0") == 0);
        }
        CHECK(tj_move_joints(2, q, 10, 10, 244, tj_now_ns()+100000000ULL) == -3);
        CHECK(tj_move_joints(0, nullptr, 10, 10, 244, tj_now_ns()+100000000ULL) == -3);
        CHECK(tj_move_joints(0, q, 0, 10, 244, tj_now_ns()+100000000ULL) == -3);
        CHECK(tj_move_joints(0, q, 10, 101, 244, tj_now_ns()+100000000ULL) == -3);
        CHECK(tj_move_joints(0, q, 10, 10, 244, tj_now_ns()-1) == -3);
        controller.set_mode(0, 1, 0, 1);
        for (int attempt = 0; attempt < 100; ++attempt) {
            if (tj_latest(&value) == 1 && value.state[0] == 1) break;
            std::this_thread::sleep_for(1ms);
        }
        CHECK(value.state[0] == 1);
        CHECK(tj_submit(1, q, 244, tj_now_ns()+100000000ULL) == 0);
        wait_send(244);
        // Even before low-speed clears, a new position target must be stopped.
        controller.stop_return_code = 1;
        CHECK(tj_hold(1, 245) == -1);
        outcome = wait_send(245);
        CHECK(std::strcmp(reinterpret_cast<const char *>(outcome.payload+12), "RSTA0") == 0);
        controller.stop_return_code = 0;
        const int file = mkstemp(path); CHECK(file >= 0); close(file); unlink(path);
        {
            LoopbackFileServer server(false);
            CHECK(tj_download_config(path) == 0);
            std::ifstream input(path); std::string line; std::getline(input, line);
            CHECK(line == "[R.A0.BASIC]");
        }
        unlink(path);
        {
            LoopbackFileServer server(true);
            CHECK(tj_download_config(path) == -1);
            CHECK(access(path, F_OK) != 0);
            CHECK(access((std::string(path) + ".partial").c_str(), F_OK) != 0);
        }
        {
            CFileOp writer; char full[] = "/dev/full", remote[] = "robot.ini";
            FileIns *request = writer.OnRecvFile(full, remote); CHECK(request != nullptr); free(request);
            FileIns response{}; response.m_InsType = INS_TYPE_Get_File_Cell_Report;
            response.m_TotalBlockNum = 1; response.m_CellSize = 1; response.m_CellContent[0] = 'x';
            writer.OnIns(&response);
            CHECK(writer.OnCheckStateOK() && writer.OnCheckErrorTag());
        }
        CHECK(tj_close() == 0 && tj_close() == 0);
        while (tj_poll_send(&outcome) == 1) {}
        const unsigned char invalid_fd_payload[] = {'F', 'Y'};
        tj_hook_send(-1, invalid_fd_payload, sizeof invalid_fd_payload, nullptr, 0);
        outcome = wait_send(0);
        CHECK(outcome.attempted == 1 && outcome.result == -1 && outcome.error_number == EBADF);

        // Direct injection tests the bounded native queue without relying on
        // kernel UDP buffering to preserve a deliberately oversized burst.
        while (tj_poll_feedback(&value) == 1) {}
        TjStats before{}; tj_stats(&before);
        DCSS packet{}; unsigned char bytes[sizeof packet + 2] = {'F', 'X'};
        for (int i = 0; i < 1030; ++i) {
            packet.m_Out[0].m_OutFrameSerial = i;
            std::memcpy(bytes + 2, &packet, sizeof packet);
            tj_hook_received(bytes, sizeof bytes);
        }
        tj_hook_received(bytes, sizeof bytes - 1);
        TjStats after{}; CHECK(tj_stats(&after) == 0);
        CHECK(after.received - before.received == 1030);
        CHECK(after.feedback_dropped - before.feedback_dropped == 6);
        CHECK(after.first_feedback_lost == before.received + 1024);
        CHECK(after.last_feedback_lost == before.received + 1029);
        int count = 0; while (tj_poll_feedback(&value) == 1) ++count;
        CHECK(count == 1024);
        CHECK(tj_latest(&value) == 1 && value.sequence[0] == 1029);
        while (tj_poll_send(&outcome) == 1) {}
        tj_stats(&before);
        for (int i = 0; i < 1030; ++i) tj_hook_send(-1, bytes, 2, nullptr, 0);
        tj_stats(&after);
        CHECK(after.send_dropped - before.send_dropped == 6);
        std::cout << "Tianji native loopback and queue tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << " (" << tj_last_error() << ")\n";
        tj_close(); return 1;
    }
}
