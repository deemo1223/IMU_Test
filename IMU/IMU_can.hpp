#ifndef IMU_CAN_H_
#define IMU_CAN_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <linux/can.h>
#include <mutex>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "imu/dataStructIMU.h"

class IMU_can
{
public:
    static constexpr int kImuCount = 4;
    static constexpr int kDataCount = 16;

    IMU_can(const char *can_iface_name, int num = 0);
    ~IMU_can();

    IMU_can(const IMU_can &) = delete;
    IMU_can &operator=(const IMU_can &) = delete;

    int can_write();
    int can_read();
    void run();
    void stop();

    void read_imu_data(float (&out)[kImuCount][kDataCount]);
    void set_can_imu_data();
    imu_data get_imu_data(int imu_id) const;
    bool is_imu_ready(int imu_id) const;
    uint8_t get_ready_mask(int imu_id) const;
    uint64_t get_received_frame_count() const;
    int get_last_can_id() const;

    float getRoll(int16_t imu_id) const;
    float getPitch(int16_t imu_id) const;
    float getYaw(int16_t imu_id) const;
    float getGyroX(int16_t imu_id) const;
    float getGyroY(int16_t imu_id) const;
    float getGyroZ(int16_t imu_id) const;
    float getAccX(int16_t imu_id) const;
    float getAccY(int16_t imu_id) const;
    float getAccZ(int16_t imu_id) const;
    float getQuarternionQ0(int16_t imu_id) const;
    float getQuarternionQ1(int16_t imu_id) const;
    float getQuarternionQ2(int16_t imu_id) const;
    float getQuarternionQ3(int16_t imu_id) const;

    can_frame rx_msg{};
    can_frame tx_msg{};

private:
    static float decode_signed_15bit(uint8_t high, uint8_t low, float scale);
    static void copy_frame_data(can_frame &dst, const can_frame &src, int bytes);

    void handle_frame(const can_frame &frame);

    int can_number = 0;
    int socket_fd = -1;
    std::atomic<bool> running{false};
    std::atomic<uint64_t> received_frame_count{0};
    std::atomic<int> last_can_id{-1};

    mutable std::mutex data_mutex;
    std::array<std::array<can_frame, 5>, kImuCount> raw_msgs{};
    float imudata_arr[kImuCount][kDataCount]{};
    imu_data can_imu_data[kImuCount]{};
    bool ready_flags[kImuCount][5]{};
};

#endif
