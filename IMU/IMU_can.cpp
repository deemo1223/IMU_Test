#include "IMU_can.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <sys/time.h>
#include <unordered_set>

namespace
{
constexpr int kRpyIndex = 0;
constexpr int kGyroIndex = 1;
constexpr int kAccIndex = 2;
constexpr int kQuatIndex = 3;
constexpr int kFreeAccIndex = 4;

int imu_index_from_can_id(int can_id)
{
    if (can_id >= 1 && can_id <= 12) {
        return (can_id - 1) / 3;
    }
    switch (can_id) {
    case 33:
    case 0x35:
        return 0;
    case 34:
    case 0x36:
        return 1;
    case 0x40:
    case 0x37:
        return 2;
    case 36:
    case 0x38:
        return 3;
    default:
        return -1;
    }
}

int data_index_from_can_id(int can_id)
{
    if (can_id >= 1 && can_id <= 12) {
        return (can_id - 1) % 3;
    }
    if (can_id == 33 || can_id == 34 || can_id == 0x40 || can_id == 36) {
        return kQuatIndex;
    }
    if (can_id >= 0x35 && can_id <= 0x38) {
        return kFreeAccIndex;
    }
    return -1;
}
}

IMU_can::IMU_can(const char *can_iface_name, int num) : can_number(num)
{
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        throw std::runtime_error("Error while opening CAN socket: " + std::string(std::strerror(errno)));
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd);
        socket_fd = -1;
        throw std::runtime_error("Error finding CAN interface " + std::string(can_iface_name) + ": " + std::string(std::strerror(errno)));
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        close(socket_fd);
        socket_fd = -1;
        throw std::runtime_error("Error binding CAN interface " + std::string(can_iface_name) + ": " + std::string(std::strerror(errno)));
    }

    struct timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        close(socket_fd);
        socket_fd = -1;
        throw std::runtime_error("Error setting CAN read timeout: " + std::string(std::strerror(errno)));
    }

    for (auto &imu_msgs : raw_msgs) {
        imu_msgs[kRpyIndex].can_dlc = 6;
        imu_msgs[kGyroIndex].can_dlc = 6;
        imu_msgs[kAccIndex].can_dlc = 6;
        imu_msgs[kQuatIndex].can_dlc = 8;
        imu_msgs[kFreeAccIndex].can_dlc = 6;
    }

    std::printf("CAN IMU interface opened: %s, socket fd: %d\n", can_iface_name, socket_fd);
}

IMU_can::~IMU_can()
{
    stop();
    if (socket_fd >= 0) {
        close(socket_fd);
    }
}

void IMU_can::stop()
{
    running = false;
}

int IMU_can::can_read()
{
    const ssize_t nbytes = read(socket_fd, &rx_msg, sizeof(struct can_frame));
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return -1;
        }
        if (running) {
            std::perror("IMU_can::can_read");
        }
        return -1;
    }
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
        return -1;
    }
    return 0;
}

int IMU_can::can_write()
{
    const ssize_t nbytes = write(socket_fd, &tx_msg, sizeof(struct can_frame));
    if (nbytes < 0) {
        std::fprintf(stderr, "IMU_can::can_write(): errno = %d [%s]\n", errno, std::strerror(errno));
        return -1;
    }
    return nbytes < static_cast<ssize_t>(sizeof(struct can_frame)) ? -1 : 0;
}

void IMU_can::run()
{
    running = true;
    std::unordered_set<int> seen_ids;

    while (running) {
        if (can_read() != 0) {
            continue;
        }

        const int can_id = static_cast<int>(rx_msg.can_id & CAN_EFF_MASK);
        received_frame_count.fetch_add(1, std::memory_order_relaxed);
        last_can_id.store(can_id, std::memory_order_relaxed);
        seen_ids.insert(can_id);
        handle_frame(rx_msg);

        static long count = 0;
        if ((++count % 200) == 1) {
            for (int imu = 1; imu <= kImuCount; ++imu) {
                if (is_imu_ready(imu)) {
                    std::printf("IMU %d ready\n", imu);
                }
            }
        }
    }
}

void IMU_can::handle_frame(const can_frame &frame)
{
    const int can_id = static_cast<int>(frame.can_id & CAN_EFF_MASK);
    const int imu_idx = imu_index_from_can_id(can_id);
    const int data_idx = data_index_from_can_id(can_id);
    if (imu_idx < 0 || data_idx < 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex);
        copy_frame_data(raw_msgs[imu_idx][data_idx], frame, data_idx == kQuatIndex ? 8 : 6);
        ready_flags[imu_idx][data_idx] = true;
        read_imu_data(imudata_arr);
        set_can_imu_data();
    }
}

void IMU_can::copy_frame_data(can_frame &dst, const can_frame &src, int bytes)
{
    dst.can_id = src.can_id;
    dst.can_dlc = src.can_dlc;
    for (int i = 0; i < bytes; ++i) {
        dst.data[i] = src.data[i];
    }
}

float IMU_can::decode_signed_15bit(uint8_t high, uint8_t low, float scale)
{
    const uint8_t sign = high >> 7;
    uint16_t magnitude = 0;
    if (sign == 1) {
        magnitude = 32768 - (((high & 0x7f) << 8) | low);
    } else {
        magnitude = ((high & 0x7f) << 8) | low;
    }
    return (sign ? -1.0f : 1.0f) * static_cast<float>(magnitude) * scale;
}

void IMU_can::read_imu_data(float (&out)[kImuCount][kDataCount])
{
    for (int imu = 0; imu < kImuCount; ++imu) {
        const auto &rpy = raw_msgs[imu][kRpyIndex].data;
        const auto &gyro = raw_msgs[imu][kGyroIndex].data;
        const auto &acc = raw_msgs[imu][kAccIndex].data;
        const auto &quat = raw_msgs[imu][kQuatIndex].data;
        const auto &free_acc = raw_msgs[imu][kFreeAccIndex].data;

        out[imu][0] = decode_signed_15bit(rpy[0], rpy[1], 0.0078f);
        out[imu][1] = decode_signed_15bit(rpy[2], rpy[3], 0.0078f);
        out[imu][2] = decode_signed_15bit(rpy[4], rpy[5], 0.0078f);

        out[imu][3] = decode_signed_15bit(gyro[0], gyro[1], 0.002f);
        out[imu][4] = decode_signed_15bit(gyro[2], gyro[3], 0.002f);
        out[imu][5] = decode_signed_15bit(gyro[4], gyro[5], 0.002f);

        out[imu][6] = decode_signed_15bit(acc[0], acc[1], 0.0039f);
        out[imu][7] = decode_signed_15bit(acc[2], acc[3], 0.0039f);
        out[imu][8] = decode_signed_15bit(acc[4], acc[5], 0.0039f);

        out[imu][9] = decode_signed_15bit(quat[0], quat[1], 3.0519e-05f);
        out[imu][10] = decode_signed_15bit(quat[2], quat[3], 3.0519e-05f);
        out[imu][11] = decode_signed_15bit(quat[4], quat[5], 3.0519e-05f);
        out[imu][12] = decode_signed_15bit(quat[6], quat[7], 3.0519e-05f);

        out[imu][13] = decode_signed_15bit(free_acc[0], free_acc[1], 0.0039f);
        out[imu][14] = decode_signed_15bit(free_acc[2], free_acc[3], 0.0039f);
        out[imu][15] = decode_signed_15bit(free_acc[4], free_acc[5], 0.0039f);
    }
}

void IMU_can::set_can_imu_data()
{
    if (can_number != 0) {
        return;
    }

    for (int imu = 0; imu < kImuCount; ++imu) {
        for (int i = 0; i < 3; ++i) {
            can_imu_data[imu].rpy[i] = imudata_arr[imu][i];
            can_imu_data[imu].gyro[i] = imudata_arr[imu][i + 3];
            can_imu_data[imu].acc[i] = imudata_arr[imu][i + 6];
            can_imu_data[imu].freeAcc[i] = imudata_arr[imu][i + 13];
        }
        for (int i = 0; i < 4; ++i) {
            can_imu_data[imu].quat[i] = imudata_arr[imu][i + 9];
        }
    }
}

imu_data IMU_can::get_imu_data(int imu_id) const
{
    if (imu_id < 1 || imu_id > kImuCount) {
        throw std::out_of_range("imu_id must be 1..4");
    }
    std::lock_guard<std::mutex> lock(data_mutex);
    return can_imu_data[imu_id - 1];
}

bool IMU_can::is_imu_ready(int imu_id) const
{
    if (imu_id < 1 || imu_id > kImuCount) {
        return false;
    }
    std::lock_guard<std::mutex> lock(data_mutex);
    const int idx = imu_id - 1;
    return ready_flags[idx][kRpyIndex] && ready_flags[idx][kGyroIndex] &&
           ready_flags[idx][kAccIndex] && ready_flags[idx][kQuatIndex];
}

uint8_t IMU_can::get_ready_mask(int imu_id) const
{
    if (imu_id < 1 || imu_id > kImuCount) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(data_mutex);
    const int idx = imu_id - 1;
    uint8_t mask = 0;
    for (int i = 0; i < 5; ++i) {
        if (ready_flags[idx][i]) {
            mask |= static_cast<uint8_t>(1u << i);
        }
    }
    return mask;
}

uint64_t IMU_can::get_received_frame_count() const
{
    return received_frame_count.load(std::memory_order_relaxed);
}

int IMU_can::get_last_can_id() const
{
    return last_can_id.load(std::memory_order_relaxed);
}

float IMU_can::getRoll(int16_t imu_id) const { return get_imu_data(imu_id).rpy[0]; }
float IMU_can::getPitch(int16_t imu_id) const { return get_imu_data(imu_id).rpy[1]; }
float IMU_can::getYaw(int16_t imu_id) const { return get_imu_data(imu_id).rpy[2]; }
float IMU_can::getGyroX(int16_t imu_id) const { return get_imu_data(imu_id).gyro[0]; }
float IMU_can::getGyroY(int16_t imu_id) const { return get_imu_data(imu_id).gyro[1]; }
float IMU_can::getGyroZ(int16_t imu_id) const { return get_imu_data(imu_id).gyro[2]; }
float IMU_can::getAccX(int16_t imu_id) const { return get_imu_data(imu_id).acc[0]; }
float IMU_can::getAccY(int16_t imu_id) const { return get_imu_data(imu_id).acc[1]; }
float IMU_can::getAccZ(int16_t imu_id) const { return get_imu_data(imu_id).acc[2]; }
float IMU_can::getQuarternionQ0(int16_t imu_id) const { return get_imu_data(imu_id).quat[0]; }
float IMU_can::getQuarternionQ1(int16_t imu_id) const { return get_imu_data(imu_id).quat[1]; }
float IMU_can::getQuarternionQ2(int16_t imu_id) const { return get_imu_data(imu_id).quat[2]; }
float IMU_can::getQuarternionQ3(int16_t imu_id) const { return get_imu_data(imu_id).quat[3]; }
