#include "can_interface.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace {

bool runCommand(const std::string& command) {
    const int ret = std::system(command.c_str());
    if (ret != 0) {
        std::cerr << "Command failed: " << command << std::endl;
        return false;
    }
    return true;
}

}  // namespace

CANInterface::CANInterface(const std::string& interface_name, int bitrate, int restart_ms)
    : interface_name_(interface_name),
      bitrate_(bitrate),
      restart_ms_(restart_ms),
      socket_fd_(-1) {
    if (bitrate_ <= 0) {
        throw std::runtime_error("Invalid CAN bitrate: " + std::to_string(bitrate_));
    }

    if (!configureInterface()) {
        throw std::runtime_error("Failed to configure CAN interface " + interface_name_);
    }

    if (!openSocket()) {
        bringInterfaceDown();
        throw std::runtime_error("Failed to open SocketCAN interface " + interface_name_);
    }

    std::cout << "CAN interface " << interface_name_ << " is ready at "
              << bitrate_ << " bps" << std::endl;
}

CANInterface::~CANInterface() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    bringInterfaceDown();
}

bool CANInterface::sendFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    if (socket_fd_ < 0) {
        std::cerr << "CAN socket is not open." << std::endl;
        return false;
    }

    if (data.size() > CAN_MAX_DLEN) {
        std::cerr << "CAN payload too large: " << data.size() << " bytes" << std::endl;
        return false;
    }

    struct can_frame frame = {};
    frame.can_id = can_id & CAN_SFF_MASK;
    frame.can_dlc = static_cast<__u8>(data.size());
    std::memcpy(frame.data, data.data(), data.size());

    const ssize_t written = write(socket_fd_, &frame, sizeof(frame));
    if (written != static_cast<ssize_t>(sizeof(frame))) {
        std::cerr << "Failed to send CAN frame on " << interface_name_
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool CANInterface::readFrame(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms) {
    if (socket_fd_ < 0) {
        std::cerr << "CAN socket is not open." << std::endl;
        return false;
    }

    if (!setReadTimeout(timeout_ms)) {
        return false;
    }

    struct can_frame frame = {};
    const ssize_t bytes_read = read(socket_fd_, &frame, sizeof(frame));
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return false;
        }

        std::cerr << "Failed to read CAN frame on " << interface_name_
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
        std::cerr << "Short CAN frame read: " << bytes_read << " bytes" << std::endl;
        return false;
    }

    can_id = frame.can_id & CAN_SFF_MASK;
    data.assign(frame.data, frame.data + frame.can_dlc);
    return true;
}

bool CANInterface::configureInterface() const {
    std::ostringstream down_cmd;
    down_cmd << "ip link set " << interface_name_ << " down";
    runCommand(down_cmd.str());

    std::ostringstream up_cmd;
    up_cmd << "ip link set " << interface_name_
           << " up type can bitrate " << bitrate_
           << " restart-ms " << restart_ms_;

    if (!runCommand(up_cmd.str())) {
        std::cerr << "Unable to bring up " << interface_name_
                  << ". Try running with sudo on Ubuntu." << std::endl;
        return false;
    }

    return true;
}

bool CANInterface::bringInterfaceDown() const {
    std::ostringstream down_cmd;
    down_cmd << "ip link set " << interface_name_ << " down";
    return runCommand(down_cmd.str());
}

bool CANInterface::openSocket() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "socket() failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    const int loopback = 0;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0) {
        std::cerr << "Failed to disable CAN loopback: " << std::strerror(errno) << std::endl;
    }

    struct ifreq ifr = {};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "ioctl(SIOCGIFINDEX) failed for " << interface_name_
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind() failed for " << interface_name_
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool CANInterface::setReadTimeout(int timeout_ms) {
    if (timeout_ms < 0) {
        std::cerr << "Invalid read timeout: " << timeout_ms << " ms" << std::endl;
        return false;
    }

    struct timeval timeout = {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        std::cerr << "Failed to set CAN read timeout: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}
