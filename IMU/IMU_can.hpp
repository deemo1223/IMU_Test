#ifndef IMU_CAN_H_
#define IMU_CAN_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "imu/dataStructIMU.h"
#include "math/cTypes.h"

class IMU_can
{
public:
    // Construct an empty single-sensor decoder.
    IMU_can();

    // Decode one raw CAN frame and merge it into the cached IMU sample.
    bool handle_frame(uint32_t can_id, const std::vector<uint8_t>& payload);

    // Return the latest cached IMU sample.
    imu_data get_data() const;
    // Report whether the required frame groups for a full sample were received.
    bool has_complete_sample() const;
    // Return a bit mask that shows which frame groups were received.
    u8 get_received_fields_mask() const;
    // Return transport-level receive statistics for diagnostics.
    u64 get_received_frame_count() const;
    // Return the most recent CAN id seen by the decoder.
    int get_last_can_id() const;
    // Return how many accepted frame updates were applied.
    u64 get_update_count() const;

private:
    enum FieldIndex {
        kRpyIndex = 0,
        kGyroIndex = 1,
        kAccIndex = 2,
        kQuatIndex = 3,
        kFreeAccIndex = 4,
        kFieldCount = 5
    };

    struct FieldMapping {
        uint32_t can_id;
        int field_index;
        std::size_t payload_size;
    };

    struct DecoderState {
        imu_data data{};
        bool received_fields[kFieldCount]{};
        u64 update_count = 0;
    };

    // Convert one signed 15-bit fixed-point field into floating-point units.
    static float decode_value(u8 high, u8 low, float scale);
    // Look up which field type belongs to a raw CAN id.
    static const FieldMapping* find_field_mapping(uint32_t can_id);
    // Decode one typed payload block into the cached sample fields.
    void decode_payload_block(int field_index, const std::vector<uint8_t>& payload);

    DecoderState state_{};
    u64 received_frame_count_ = 0;
    int last_can_id_ = -1;
};

#endif
