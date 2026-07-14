#ifndef IMU_CAN_H_
#define IMU_CAN_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "imu/dataStructIMU.h"

class IMU_can
{
public:
    IMU_can();

    bool process_frame(uint32_t can_id, const std::vector<uint8_t>& data);

    imu_data get_imu_data(int imu_id) const;
    bool is_imu_ready(int imu_id) const;
    uint8_t get_ready_mask(int imu_id) const;
    uint64_t get_received_frame_count() const;
    int get_last_can_id() const;
    uint64_t get_update_count(int imu_id) const;

private:
    enum DataIndex {
        kRpyIndex = 0,
        kGyroIndex = 1,
        kAccIndex = 2,
        kQuatIndex = 3,
        kFreeAccIndex = 4,
        kFrameTypeCount = 5
    };

    struct SensorState {
        imu_data data{};
        bool ready_flags[kFrameTypeCount]{};
        uint64_t update_count = 0;
    };

    static float decode_signed_15bit(uint8_t high, uint8_t low, float scale);
    static bool match_frame(uint32_t can_id, int& imu_index, int& data_index);
    static std::size_t expected_payload_size(int data_index);

    SensorState* find_sensor(int imu_id);
    const SensorState* find_sensor(int imu_id) const;
    void parseResponse(SensorState& sensor, int data_index, const std::vector<uint8_t>& data);

    std::unordered_map<int, SensorState> sensors_;
    uint64_t received_frame_count_ = 0;
    int last_can_id_ = -1;
};

#endif
