#include "IMU_can.hpp"
#include "math/orientation_tools_modify.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <ctime>
#include <vector>

namespace
{
constexpr int kDefaultTorsoImuId = 3;

std::atomic<bool> g_running{true};

void signal_handler(int)
{
    g_running = false;
}

float unwrap_angle(float current, float last, int &circles, float period)
{
    const float half_period = period * 0.5f;
    if (current - last < -half_period) {
        ++circles;
    } else if (current - last > half_period) {
        --circles;
    }
    return period * static_cast<float>(circles) + current;
}

std::string shell_quote(const std::string &value)
{
    std::string quoted = "'";
    for (const char ch : value) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted += ch;
        }
    }
    quoted += "'";
    return quoted;
}

std::filesystem::path find_init_can_script(const char *argv0)
{
    const std::filesystem::path exe_path(argv0);
    const std::filesystem::path exe_dir = exe_path.has_parent_path() ? exe_path.parent_path() : ".";
    const std::vector<std::filesystem::path> candidates = {
        std::filesystem::current_path() / "IMU" / "init_can.sh",
        std::filesystem::current_path() / "scripts" / "init_can.sh",
        exe_dir / ".." / "IMU" / "init_can.sh",
        exe_dir / ".." / "scripts" / "init_can.sh",
    };

    for (const auto &candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return std::filesystem::weakly_canonical(candidate);
        }
    }
    return {};
}

bool can_interface_is_up(const std::string &can_iface)
{
    std::ifstream state_file("/sys/class/net/" + can_iface + "/operstate");
    std::string state;
    state_file >> state;
    return state == "up";
}

bool run_can_init_script(const char *argv0, const std::string &can_iface)
{
    if (can_interface_is_up(can_iface)) {
        std::printf("CAN interface %s is already up, skip init script.\n", can_iface.c_str());
        return true;
    }

    const std::filesystem::path script_path = find_init_can_script(argv0);
    if (script_path.empty()) {
        std::fprintf(stderr, "Could not find IMU/init_can.sh or scripts/init_can.sh\n");
        return false;
    }

    const std::string command = "bash " + shell_quote(script_path.string()) + " " +
                                shell_quote(can_iface) + " 1000000";
    std::printf("Initializing CAN interface: %s\n", command.c_str());
    const int ret = std::system(command.c_str());
    if (ret != 0) {
        std::fprintf(stderr, "CAN init failed with return code %d\n", ret);
        return false;
    }
    return true;
}

std::string make_log_filename()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time {};
    localtime_r(&now_time, &local_time);

    std::ostringstream name;
    name << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
    return name.str();
}

std::ofstream open_log_file()
{
    const std::filesystem::path log_dir = std::filesystem::current_path() / "logs";
    std::filesystem::create_directories(log_dir);

    const std::filesystem::path log_path = log_dir / make_log_filename();
    std::ofstream log_file(log_path);
    if (!log_file.is_open()) {
        throw std::runtime_error("Failed to open log file: " + log_path.string());
    }

    log_file << "frame,t_ms,t_s,"
             << "rpy_x,rpy_y,rpy_z,"
             << "gyro_x,gyro_y,gyro_z,"
             << "acc_x,acc_y,acc_z,"
             << "quat_1,quat_2,quat_3,quat_4,"
             << "freeAcc_x,freeAcc_y\n";
    log_file << std::fixed << std::setprecision(6);
    std::printf("Logging IMU data to %s\n", log_path.string().c_str());
    return log_file;
}

void print_usage(const char *program)
{
    std::printf("Usage: %s [can_iface] [torso_imu_id]\n", program);
    std::printf("  can_iface: default can0\n");
    std::printf("  torso_imu_id: 1..4, default %d (centaur torso/com IMU in original project)\n",
                kDefaultTorsoImuId);
}
}

int main(int argc, char **argv)
{
    if (argc > 1 && (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }

    const std::string can_iface = argc > 1 ? argv[1] : "can0";
    const int torso_imu_id = argc > 2 ? std::atoi(argv[2]) : kDefaultTorsoImuId;
    if (torso_imu_id < 1 || torso_imu_id > IMU_can::kImuCount) {
        std::fprintf(stderr, "torso_imu_id must be 1..4\n");
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        if (!run_can_init_script(argv[0], can_iface)) {
            std::fprintf(stderr, "Failed to initialize %s. Check CAN adapter, sudo permission, and bitrate.\n",
                         can_iface.c_str());
            return 1;
        }

        IMU_can imu(can_iface.c_str(), 0);
        std::thread imu_thread(&IMU_can::run, &imu);

        bool first_visit = true;
        float last_yaw = 0.0f;
        int yaw_circles = 0;
        int repeat_counter = 0;
        constexpr int max_repeat = 200;
        float last_raw_acc[3] = {0.0f, 0.0f, 0.0f};
        Vec3<float> rpy_ini_0 = Vec3<float>::Zero();
        const Quat<float> q_fix(0.0f, 1.0f, 0.0f, 0.0f);
        bool first_output = true;
        auto last_diag_time = std::chrono::steady_clock::now();
        bool logging_started = false;
        uint64_t log_frame = 0;
        std::ofstream log_file;
        std::chrono::steady_clock::time_point log_start_time;

        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (!imu.is_imu_ready(torso_imu_id)) {
                const auto now = std::chrono::steady_clock::now();
                if (now - last_diag_time > std::chrono::seconds(1)) {
                    const uint64_t frame_count = imu.get_received_frame_count();
                    const int last_can_id = imu.get_last_can_id();
                    const uint8_t ready_mask = imu.get_ready_mask(torso_imu_id);
                    if (frame_count == 0) {
                        std::printf("Waiting: no CAN frames received on %s. Check IMU power, CANH/CANL, GND, termination, and bitrate.\n",
                                    can_iface.c_str());
                    } else {
                        std::printf("Waiting: received %lu CAN frames, last id=0x%X, IMU%d ready mask=0x%02X "
                                    "(bits: rpy=1 gyro=2 acc=4 quat=8 freeAcc=16)\n",
                                    frame_count, last_can_id, torso_imu_id, ready_mask);
                    }
                    last_diag_time = now;
                }
                continue;
            }

            const imu_data data = imu.get_imu_data(torso_imu_id);
            Quat<float> robot_raw_quat;
            for (int i = 0; i < 4; ++i) {
                robot_raw_quat[i] = data.quat[i];
            }
            const float quat_norm = robot_raw_quat.norm();
            if (quat_norm < 1e-4f) {
                std::printf("Waiting for valid IMU %d quaternion on %s...\r", torso_imu_id, can_iface.c_str());
                std::fflush(stdout);
                continue;
            }
            robot_raw_quat /= quat_norm;

            if (first_visit) {
                rpy_ini_0 = ori_modify::quatToRPY(robot_raw_quat);
                first_visit = false;
                std::printf("\nInitialized IMU %d yaw offset: %.4f deg\n",
                            torso_imu_id, ori_modify::rad2deg(rpy_ini_0[2]));
            }

            if (data.acc[0] == last_raw_acc[0] && data.acc[1] == last_raw_acc[1] && data.acc[2] == last_raw_acc[2]) {
                ++repeat_counter;
            } else {
                repeat_counter = 0;
            }
            for (int i = 0; i < 3; ++i) {
                last_raw_acc[i] = data.acc[i];
            }
            if (repeat_counter > max_repeat) {
                std::fprintf(stderr, "\nIMU data error: acceleration repeated for too long\n");
                repeat_counter = 0;
            }

            Vec3<float> rpy_ini_yaw = rpy_ini_0;
            rpy_ini_yaw[0] = 0.0f;
            rpy_ini_yaw[1] = 0.0f;
            const Quat<float> robot_quat_temp = ori_modify::quatProduct(robot_raw_quat, q_fix);
            const Quat<float> robot_quat = ori_modify::quatProduct(
                ori_modify::quatConjugate(ori_modify::rpyToQuat(rpy_ini_yaw)),
                robot_quat_temp);
            Vec3<float> rpy_rad = ori_modify::quatToRPY(robot_quat);
            if (first_output) {
                last_yaw = rpy_rad[2];
                first_output = false;
            }
            rpy_rad[2] = unwrap_angle(rpy_rad[2], last_yaw, yaw_circles, static_cast<float>(2.0 * M_PI));
            last_yaw = ori_modify::quatToRPY(robot_quat)[2];

            const float rpy_deg[3] = {
                ori_modify::rad2deg(rpy_rad[0]),
                ori_modify::rad2deg(rpy_rad[1]),
                ori_modify::rad2deg(rpy_rad[2]),
            };

            // Same sign convention used by stateestimator::run(): keep X, flip Y/Z.
            const float gyro_body[3] = {data.gyro[0], -data.gyro[1], -data.gyro[2]};
            const float acc_body[3] = {data.acc[0], -data.acc[1], -data.acc[2]};

            if (!logging_started) {
                log_file = open_log_file();
                log_start_time = std::chrono::steady_clock::now();
                logging_started = true;
            }

            const auto sample_time = std::chrono::steady_clock::now();
            const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                sample_time - log_start_time).count();
            const double t_ms = static_cast<double>(elapsed_us) / 1000.0;
            const double t_s = static_cast<double>(elapsed_us) / 1000000.0;
            log_file << log_frame << ','
                     << t_ms << ','
                     << t_s << ','
                     << rpy_deg[0] << ','
                     << rpy_deg[1] << ','
                     << rpy_deg[2] << ','
                     << gyro_body[0] << ','
                     << gyro_body[1] << ','
                     << gyro_body[2] << ','
                     << acc_body[0] << ','
                     << acc_body[1] << ','
                     << acc_body[2] << ','
                     << data.quat[0] << ','
                     << data.quat[1] << ','
                     << data.quat[2] << ','
                     << data.quat[3] << ','
                     << data.freeAcc[0] << ','
                     << data.freeAcc[1] << '\n';
            ++log_frame;

            std::printf(
                "IMU%d rpy[deg]=(%.4f, %.4f, %.4f) gyro=(%.4f, %.4f, %.4f) acc=(%.4f, %.4f, %.4f) "
                "quat=(%.5f, %.5f, %.5f, %.5f) freeAcc=(%.4f, %.4f, %.4f)\n",
                torso_imu_id,
                rpy_deg[0], rpy_deg[1], rpy_deg[2],
                gyro_body[0], gyro_body[1], gyro_body[2],
                acc_body[0], acc_body[1], acc_body[2],
                data.quat[0], data.quat[1], data.quat[2], data.quat[3],
                data.freeAcc[0], data.freeAcc[1], data.freeAcc[2]);
        }

        imu.stop();
        if (imu_thread.joinable()) {
            imu_thread.join();
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "IMU program failed: %s\n", e.what());
        return 1;
    }

    return 0;
}
