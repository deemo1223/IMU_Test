#include "IMU_can.hpp"

#include <stdexcept>

namespace
{
constexpr int kLegacyRpyBaseId = 1;
constexpr int kLegacyRpyFrameCount = 3;
constexpr int kLegacyFrameBlockSize = 3;

constexpr uint32_t kQuatCanIds[] = {33, 34, 0x40, 36};
constexpr uint32_t kFreeAccCanIds[] = {0x35, 0x36, 0x37, 0x38};
}

IMU_can::IMU_can() = default;

bool IMU_can::process_frame(uint32_t can_id, const std::vector<uint8_t>& data)
{
    ++received_frame_count_;
    last_can_id_ = static_cast<int>(can_id);

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

imu_data IMU_can::get_imu_data(int imu_id) const
{
    return get_sensor_or_throw(imu_id).data;
}

bool IMU_can::is_imu_ready(int imu_id) const
{
    if (imu_id < 1 || imu_id > kImuCount) {
        return false;
    }

    const SensorState& sensor = sensors_[imu_id - 1];
    return sensor.ready_flags[kRpyIndex] &&
           sensor.ready_flags[kGyroIndex] &&
           sensor.ready_flags[kAccIndex] &&
           sensor.ready_flags[kQuatIndex];
}

uint8_t IMU_can::get_ready_mask(int imu_id) const
{
    if (imu_id < 1 || imu_id > kImuCount) {
        return 0;
    }

    const SensorState& sensor = sensors_[imu_id - 1];
    uint8_t mask = 0;
    for (int index = 0; index < kFrameTypeCount; ++index) {
        if (sensor.ready_flags[index]) {
            mask |= static_cast<uint8_t>(1u << index);
        }
    }
    return mask;
}

uint64_t IMU_can::get_received_frame_count() const
{
    return received_frame_count_;
}

int IMU_can::get_last_can_id() const
{
    return last_can_id_;
}

uint64_t IMU_can::get_update_count(int imu_id) const
{
    return imu_id >= 1 && imu_id <= kImuCount ? sensors_[imu_id - 1].update_count : 0;
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

bool IMU_can::match_frame(uint32_t can_id, int& imu_index, int& data_index)
{
    if (static_cast<int>(can_id) >= kLegacyRpyBaseId &&
        static_cast<int>(can_id) < kLegacyRpyBaseId + kImuCount * kLegacyFrameBlockSize) {
        const int zero_based = (static_cast<int>(can_id) - kLegacyRpyBaseId) / kLegacyFrameBlockSize;
        imu_index = zero_based;
        data_index = (static_cast<int>(can_id) - kLegacyRpyBaseId) % kLegacyFrameBlockSize;
        return true;
    }

    for (int index = 0; index < kImuCount; ++index) {
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

std::size_t IMU_can::expected_payload_size(int data_index)
{
    return data_index == kQuatIndex ? 8 : 6;
}

IMU_can::SensorState& IMU_can::get_sensor_or_throw(int imu_id)
{
    if (imu_id < 1 || imu_id > kImuCount) {
        throw std::out_of_range("imu_id must be 1..4");
    }
    return sensors_[imu_id - 1];
}

const IMU_can::SensorState& IMU_can::get_sensor_or_throw(int imu_id) const
{
    if (imu_id < 1 || imu_id > kImuCount) {
        throw std::out_of_range("imu_id must be 1..4");
    }
    return sensors_[imu_id - 1];
}

void IMU_can::parseResponse(SensorState& sensor, int data_index, const std::vector<uint8_t>& data)
{
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
