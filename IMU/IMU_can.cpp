#include "IMU_can.hpp"

// Construct an empty single-sensor decoder.
IMU_can::IMU_can() = default;

// Decode one raw CAN frame and merge it into the cached IMU sample.
bool IMU_can::handle_frame(uint32_t can_id, const std::vector<uint8_t>& payload)
{
    ++received_frame_count_;
    last_can_id_ = static_cast<int>(can_id);

    const FieldMapping* mapping = find_field_mapping(can_id);
    if (mapping == nullptr) {
        return false;
    }

    if (payload.size() < mapping->payload_size) {
        return false;
    }

    decode_payload_block(mapping->field_index, payload);
    state_.received_fields[mapping->field_index] = true;
    ++state_.update_count;
    return true;
}

// Return the latest cached IMU sample.
imu_data IMU_can::get_data() const
{
    return state_.data;
}

// Report whether the required frame groups for a full sample were received.
bool IMU_can::has_complete_sample() const
{
    return state_.received_fields[kRpyIndex] &&
           state_.received_fields[kGyroIndex] &&
           state_.received_fields[kAccIndex] &&
           state_.received_fields[kQuatIndex];
}

// Return a bit mask that shows which frame groups were received.
u8 IMU_can::get_received_fields_mask() const
{
    u8 mask = 0;
    for (int index = 0; index < kFieldCount; ++index) {
        if (state_.received_fields[index]) {
            mask |= static_cast<u8>(1u << index);
        }
    }
    return mask;
}

// Return transport-level receive statistics for diagnostics.
u64 IMU_can::get_received_frame_count() const
{
    return received_frame_count_;
}

// Return the most recent CAN id seen by the decoder.
int IMU_can::get_last_can_id() const
{
    return last_can_id_;
}

// Return how many accepted frame updates were applied.
u64 IMU_can::get_update_count() const
{
    return state_.update_count;
}

// Convert one signed 15-bit fixed-point field into floating-point units.
float IMU_can::decode_value(u8 high, u8 low, float scale)
{
    const u8 sign = high >> 7;
    u16 magnitude = 0;
    if (sign == 1) {
        magnitude = 32768 - (((high & 0x7f) << 8) | low);
    } else {
        magnitude = ((high & 0x7f) << 8) | low;
    }
    return (sign ? -1.0f : 1.0f) * static_cast<float>(magnitude) * scale;
}

// Return the fixed CAN-id to payload-type mapping for the connected IMU.
const std::vector<IMU_can::FieldMapping>& IMU_can::field_mappings()
{
    static const std::vector<FieldMapping> mappings = {
        {0x001, kRpyIndex, 6},
        {0x002, kGyroIndex, 6},
        {0x003, kAccIndex, 6},
        {0x021, kQuatIndex, 8},
        {0x035, kFreeAccIndex, 6},
    };
    return mappings;
}

// Look up which field type belongs to a raw CAN id.
const IMU_can::FieldMapping* IMU_can::find_field_mapping(uint32_t can_id)
{
    for (const FieldMapping& mapping : field_mappings()) {
        if (mapping.can_id == can_id) {
            return &mapping;
        }
    }
    return nullptr;
}

// Decode one typed payload block into the cached sample fields.
void IMU_can::decode_payload_block(int field_index, const std::vector<uint8_t>& payload)
{
    switch (field_index) {
    case kRpyIndex:
        state_.data.rpy[0] = decode_value(payload[0], payload[1], 0.0078f);
        state_.data.rpy[1] = decode_value(payload[2], payload[3], 0.0078f);
        state_.data.rpy[2] = decode_value(payload[4], payload[5], 0.0078f);
        break;
    case kGyroIndex:
        state_.data.gyro[0] = decode_value(payload[0], payload[1], 0.002f);
        state_.data.gyro[1] = decode_value(payload[2], payload[3], 0.002f);
        state_.data.gyro[2] = decode_value(payload[4], payload[5], 0.002f);
        break;
    case kAccIndex:
        state_.data.acc[0] = decode_value(payload[0], payload[1], 0.0039f);
        state_.data.acc[1] = decode_value(payload[2], payload[3], 0.0039f);
        state_.data.acc[2] = decode_value(payload[4], payload[5], 0.0039f);
        break;
    case kQuatIndex:
        state_.data.quat[0] = decode_value(payload[0], payload[1], 3.0519e-05f);
        state_.data.quat[1] = decode_value(payload[2], payload[3], 3.0519e-05f);
        state_.data.quat[2] = decode_value(payload[4], payload[5], 3.0519e-05f);
        state_.data.quat[3] = decode_value(payload[6], payload[7], 3.0519e-05f);
        break;
    case kFreeAccIndex:
        state_.data.freeAcc[0] = decode_value(payload[0], payload[1], 0.0039f);
        state_.data.freeAcc[1] = decode_value(payload[2], payload[3], 0.0039f);
        state_.data.freeAcc[2] = decode_value(payload[4], payload[5], 0.0039f);
        break;
    default:
        break;
    }
}
