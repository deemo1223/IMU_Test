#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <cstdint>
#include <string>
#include <vector>

class CANInterface {
public:
    CANInterface(const std::string& interface_name, int bitrate, int restart_ms = 100);
    ~CANInterface();

    CANInterface(const CANInterface&) = delete;
    CANInterface& operator=(const CANInterface&) = delete;

    bool sendFrame(uint32_t can_id, const std::vector<uint8_t>& data);
    bool readFrame(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms);

private:
    bool configureInterface() const;
    bool bringInterfaceDown() const;
    bool openSocket();
    bool setReadTimeout(int timeout_ms);

    std::string interface_name_;
    int bitrate_;
    int restart_ms_;
    int socket_fd_;
};

#endif
