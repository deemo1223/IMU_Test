#include "IMU_can.hpp"
#include "can_interface.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
struct CommandLineOptions {
    std::string interface_name = "can0";
    int bitrate = 1000000;
    uint8_t sensor_id = 3;
};

constexpr int kReadTimeoutMs = 5;
constexpr auto kPrintPeriod = std::chrono::milliseconds(100);

std::atomic<bool> g_running{true};

void signal_handler(int)
{
    g_running = false;
}

[[noreturn]] void print_usage_and_exit(const char* program_name, int exit_code)
{
    FILE* stream = exit_code == 0 ? stdout : stderr;
    std::fprintf(stream,
                 "Usage: %s [--can <interface> | --can0 | --can1 ...] "
                 "[--bitrate <bps>] "
                 "[--sensor-id <id> | --sensorID <id> | --sensorID<id>] "
                 "[--help]\n",
                 program_name);
    std::exit(exit_code);
}

uint8_t parse_sensor_id(const std::string& value)
{
    const unsigned long parsed = std::stoul(value, nullptr, 0);
    if (parsed < 1 || parsed > IMU_can::kImuCount) {
        throw std::out_of_range("sensor ID must be in range 1..4");
    }
    if (parsed > std::numeric_limits<uint8_t>::max()) {
        throw std::out_of_range("sensor ID must fit in one byte");
    }
    return static_cast<uint8_t>(parsed);
}

CommandLineOptions parse_arguments(int argc, char* argv[])
{
    CommandLineOptions options;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];

        if (argument == "--help" || argument == "-h") {
            print_usage_and_exit(argv[0], 0);
        }

        if (argument == "--can") {
            if (index + 1 >= argc) {
                throw std::invalid_argument("--can requires an interface name");
            }
            options.interface_name = argv[++index];
            continue;
        }

        if (argument.rfind("--can", 0) == 0 && argument.size() > 5) {
            options.interface_name = argument.substr(2);
            continue;
        }

        if (argument == "--bitrate") {
            if (index + 1 >= argc) {
                throw std::invalid_argument("--bitrate requires a numeric value");
            }
            options.bitrate = std::stoi(argv[++index], nullptr, 0);
            continue;
        }

        if (argument == "--sensor-id" || argument == "--sensorID") {
            if (index + 1 >= argc) {
                throw std::invalid_argument(argument + " requires a sensor ID value");
            }
            options.sensor_id = parse_sensor_id(argv[++index]);
            continue;
        }

        if (argument.rfind("--sensorID", 0) == 0 && argument.size() > 10) {
            options.sensor_id = parse_sensor_id(argument.substr(10));
            continue;
        }

        throw std::invalid_argument("Unknown argument: " + argument);
    }

    return options;
}

bool pump_can_frames(CANInterface& can, IMU_can& imu)
{
    bool received_any = false;
    while (g_running) {
        uint32_t can_id = 0;
        std::vector<uint8_t> data;
        if (!can.readFrame(can_id, data, kReadTimeoutMs)) {
            break;
        }

        received_any = true;
        imu.process_frame(can_id, data);
    }
    return received_any;
}
}

int main(int argc, char* argv[])
{
    try {
        const CommandLineOptions options = parse_arguments(argc, argv);

        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);

        CANInterface can(options.interface_name, options.bitrate);
        IMU_can imu;

        auto last_diag_time = std::chrono::steady_clock::now();
        auto last_print_time = std::chrono::steady_clock::now();

        while (g_running) {
            const bool received_any = pump_can_frames(can, imu);
            if (!imu.is_imu_ready(options.sensor_id)) {
                const auto now = std::chrono::steady_clock::now();
                if (now - last_diag_time > std::chrono::seconds(1)) {
                    const uint64_t frame_count = imu.get_received_frame_count();
                    const int last_can_id = imu.get_last_can_id();
                    const uint8_t ready_mask = imu.get_ready_mask(options.sensor_id);
                    if (frame_count == 0) {
                        std::printf("Waiting: no CAN frames received on %s. Check IMU power, CANH/CANL, GND, termination, and bitrate.\n",
                                    options.interface_name.c_str());
                    } else {
                        std::printf("Waiting: received %lu CAN frames, last id=0x%X, IMU%d ready mask=0x%02X "
                                    "(bits: rpy=1 gyro=2 acc=4 quat=8 freeAcc=16)\n",
                                    frame_count, last_can_id, options.sensor_id, ready_mask);
                    }
                    last_diag_time = now;
                }

                if (!received_any) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                continue;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_print_time < kPrintPeriod) {
                if (!received_any) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                continue;
            }
            last_print_time = now;

            const imu_data data = imu.get_imu_data(options.sensor_id);
            std::printf("IMU%d rpy[deg]=(%.4f, %.4f, %.4f)\n",
                        options.sensor_id,
                        data.rpy[0],
                        data.rpy[1],
                        data.rpy[2]);
        }
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "IMU program failed: %s\n", exception.what());
        return 1;
    }

    return 0;
}
