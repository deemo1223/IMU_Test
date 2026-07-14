#include "IMU_can.hpp"

#include <stdexcept>

namespace
{
constexpr int kLegacyRpyBaseId = 1;
constexpr int kLegacyFrameBlockSize = 3;

constexpr uint32_t kQuatCanIds[] = {33, 34, 0x40, 36};
constexpr uint32_t kFreeAccCanIds[] = {0x35, 0x36, 0x37, 0x38};
}

IMU_can::IMU_can() = default;

// Accept one CAN frame, classify it, and fold its payload into cached IMU state.
bool IMU_can::process_frame(uint32_t can_id, const std::vector<uint8_t>& data)
{
    ++received_frame_count_;
    last_can_id_ = static_cast<int>(can_id);

    // First classify the incoming frame, then update only the matching sensor state.
    int imu_index = -1;
    int data_index = -1;
    if (!match_frame(can_id, imu_index, data_index)) {
        return false;
    }

    if (data.size() < expected_payload_size(data_index)) {
        return false;
    }

    SensorState& sensor = sensors_[imu_index];
    parseResponse(sensor, data_index, data);
    sensor.ready_flags[data_index] = true;
    ++sensor.update_count;
    return true;
}

// Return the latest decoded sample for one sensor id.
imu_data IMU_can::get_imu_data(int imu_id) const
{
    const SensorState* sensor = find_sensor(imu_id);
    if (sensor == nullptr) {
        throw std::out_of_range("imu_id not available");
    }
    return sensor->data;
}

// Report whether the sensor has received the frame types needed for RPY output.
bool IMU_can::is_imu_ready(int imu_id) const
{
    const SensorState* sensor = find_sensor(imu_id);
    if (sensor == nullptr) {
        return false;
    }

    return sensor->ready_flags[kRpyIndex] &&
           sensor->ready_flags[kGyroIndex] &&
           sensor->ready_flags[kAccIndex] &&
           sensor->ready_flags[kQuatIndex];
}

// Expose which frame groups have been received as a compact bit mask.
uint8_t IMU_can::get_ready_mask(int imu_id) const
{
    const SensorState* sensor = find_sensor(imu_id);
    if (sensor == nullptr) {
        return 0;
    }

    uint8_t mask = 0;
    for (int index = 0; index < kFrameTypeCount; ++index) {
        if (sensor->ready_flags[index]) {
            mask |= static_cast<uint8_t>(1u << index);
        }
    }
    return mask;
}

// Return the total number of CAN frames observed by this decoder.
uint64_t IMU_can::get_received_frame_count() const
{
    return received_frame_count_;
}

// Return the most recent CAN id seen on the bus.
int IMU_can::get_last_can_id() const
{
    return last_can_id_;
}

// Return how many accepted updates have been applied to one sensor state.
uint64_t IMU_can::get_update_count(int imu_id) const
{
    const SensorState* sensor = find_sensor(imu_id);
    return sensor == nullptr ? 0 : sensor->update_count;
}

// Decode the IMU's signed 15-bit fixed-point payload format into float units.
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

// Map a raw CAN id to a logical sensor index and frame type.
bool IMU_can::match_frame(uint32_t can_id, int& imu_index, int& data_index)
{
    if (static_cast<int>(can_id) >= kLegacyRpyBaseId &&
        static_cast<int>(can_id) < kLegacyRpyBaseId +
                                     static_cast<int>(std::size(kQuatCanIds)) * kLegacyFrameBlockSize) {
        const int zero_based = (static_cast<int>(can_id) - kLegacyRpyBaseId) / kLegacyFrameBlockSize;
        imu_index = zero_based;
        data_index = (static_cast<int>(can_id) - kLegacyRpyBaseId) % kLegacyFrameBlockSize;
        return true;
    }

    for (int index = 0; index < static_cast<int>(std::size(kQuatCanIds)); ++index) {
        if (can_id == kQuatCanIds[index]) {
            imu_index = index;
            data_index = kQuatIndex;
            return true;
        }
        if (can_id == kFreeAccCanIds[index]) {
            imu_index = index;
            data_index = kFreeAccIndex;
            return true;
        }
    }

    return false;
}

// Return the expected byte length for one logical IMU frame type.
std::size_t IMU_can::expected_payload_size(int data_index)
{
    return data_index == kQuatIndex ? 8 : 6;
}

// Look up mutable cached state for one logical sensor id.
IMU_can::SensorState* IMU_can::find_sensor(int imu_id)
{
    const auto it = sensors_.find(imu_id);
    return it == sensors_.end() ? nullptr : &it->second;
}

// Look up read-only cached state for one logical sensor id.
const IMU_can::SensorState* IMU_can::find_sensor(int imu_id) const
{
    const auto it = sensors_.find(imu_id);
    return it == sensors_.end() ? nullptr : &it->second;
}

// Decode one typed IMU payload block into the matching fields of the cached sample.
void IMU_can::parseResponse(SensorState& sensor, int data_index, const std::vector<uint8_t>& data)
{
    // Each CAN frame updates one typed block inside the cached IMU sample.
    switch (data_index) {
    case kRpyIndex:
        sensor.data.rpy[0] = decode_signed_15bit(data[0], data[1], 0.0078f);
        sensor.data.rpy[1] = decode_signed_15bit(data[2], data[3], 0.0078f);
        sensor.data.rpy[2] = decode_signed_15bit(data[4], data[5], 0.0078f);
        break;
    case kGyroIndex:
        sensor.data.gyro[0] = decode_signed_15bit(data[0], data[1], 0.002f);
        sensor.data.gyro[1] = decode_signed_15bit(data[2], data[3], 0.002f);
        sensor.data.gyro[2] = decode_signed_15bit(data[4], data[5], 0.002f);
        break;
    case kAccIndex:
        sensor.data.acc[0] = decode_signed_15bit(data[0], data[1], 0.0039f);
        sensor.data.acc[1] = decode_signed_15bit(data[2], data[3], 0.0039f);
        sensor.data.acc[2] = decode_signed_15bit(data[4], data[5], 0.0039f);
        break;
    case kQuatIndex:
        sensor.data.quat[0] = decode_signed_15bit(data[0], data[1], 3.0519e-05f);
        sensor.data.quat[1] = decode_signed_15bit(data[2], data[3], 3.0519e-05f);
        sensor.data.quat[2] = decode_signed_15bit(data[4], data[5], 3.0519e-05f);
        sensor.data.quat[3] = decode_signed_15bit(data[6], data[7], 3.0519e-05f);
        break;
    case kFreeAccIndex:
        sensor.data.freeAcc[0] = decode_signed_15bit(data[0], data[1], 0.0039f);
        sensor.data.freeAcc[1] = decode_signed_15bit(data[2], data[3], 0.0039f);
        sensor.data.freeAcc[2] = decode_signed_15bit(data[4], data[5], 0.0039f);
        break;
    default:
        break;
    }
}
