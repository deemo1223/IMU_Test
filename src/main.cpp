#include "IMU_can.hpp"
#include "can_interface.hpp"
#include "math/orientation_tools_modify.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <thread>
#include <vector>

namespace
{
constexpr const char* kCanInterface = "can0";
constexpr int kCanBitrate = 1000000;
constexpr int kReadTimeoutMs = 5;
constexpr int kMaxFramesPerCycle = 32;
constexpr auto kPrintPeriod = std::chrono::milliseconds(100);
constexpr auto kIdleSleep = std::chrono::milliseconds(20);

std::atomic<bool> g_running{true};

// Stop the polling loop cleanly on Ctrl+C or termination.
void signal_handler(int)
{
    g_running = false;
}

// Read frames until the socket times out so the latest sample stays current.
bool pump_can_frames(CANInterface& can, IMU_can& imu)
{
    bool received_any = false;
    // Process a bounded burst so printing and diagnostics still get CPU time.
    for (int frame = 0; frame < kMaxFramesPerCycle && g_running; ++frame) {
        uint32_t can_id = 0;
        std::vector<uint8_t> data;
        if (!can.readFrame(can_id, data, kReadTimeoutMs)) {
            break;
        }

        received_any = true;
        imu.handle_frame(can_id, data);
    }
    return received_any;
}
}

// Open can0, decode one hard-coded sensor, and print RPY every 100 ms.
int main()
{
    try {
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        CANInterface can(kCanInterface, kCanBitrate);
        IMU_can imu;

        auto last_diag_time = std::chrono::steady_clock::now();
        auto last_print_time = std::chrono::steady_clock::now();

        while (g_running) {
            // Keep CAN reads responsive, but print only on a slower fixed cadence.
            const bool received_any = pump_can_frames(can, imu);
            const auto now = std::chrono::steady_clock::now();

            if (!imu.has_complete_sample()) {
                if (now - last_diag_time >= std::chrono::seconds(1)) {
                    const uint64_t frame_count = imu.get_received_frame_count();
                    const int last_can_id = imu.get_last_can_id();
                    const uint8_t ready_mask = imu.get_received_fields_mask();
                    if (frame_count == 0) {
                        std::printf("Waiting: no CAN frames received on %s\n", kCanInterface);
                    } else {
                        std::printf("Waiting: received %lu CAN frames, last id=0x%X, ready mask=0x%02X\n",
                                    frame_count, last_can_id, ready_mask);
                    }
                    last_diag_time = now;
                }

                if (!received_any) {
                    std::this_thread::sleep_for(kIdleSleep);
                }
                continue;
            }

            if (now - last_print_time >= kPrintPeriod) {
                const imu_data data = imu.get_data();
                Quat<float> quat;
                quat[0] = data.quat[0];
                quat[1] = data.quat[1];
                quat[2] = data.quat[2];
                quat[3] = data.quat[3];
                const float quat_norm = quat.norm();
                if (quat_norm < 1e-6f) {
                    if (!received_any) {
                        std::this_thread::sleep_for(kIdleSleep);
                    }
                    continue;
                }
                quat /= quat_norm;
                const Vec3<float> rpy_rad = ori_modify::quatToRPY(quat);
                std::printf("IMU rpy[deg]=(%.4f, %.4f, %.4f)\n",
                            ori_modify::rad2deg(rpy_rad[0]),
                            ori_modify::rad2deg(rpy_rad[1]),
                            ori_modify::rad2deg(rpy_rad[2]));
                last_print_time = now;
            } else if (!received_any) {
                std::this_thread::sleep_for(kIdleSleep);
            }
        }
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "IMU program failed: %s\n", exception.what());
        return 1;
    }

    return 0;
}
