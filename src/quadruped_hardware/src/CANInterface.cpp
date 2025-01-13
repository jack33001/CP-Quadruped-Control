#include "quadruped_hardware/CANInterface.hpp"


namespace CAN_interface
{
    CANInterface::CANInterface(const char* socketName)
    {
        struct ifreq ifr; // Interface Request structure
        int loopback = 0; /* 0 = disabled, 1 = enabled (default) */

        if ((socket_descrp_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("CANInterface: Error While Opening CAN Socket");
            return;
        }

        setsockopt(socket_descrp_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
        strcpy(ifr.ifr_name, socketName);
        ioctl(socket_descrp_, SIOCGIFINDEX, &ifr);

        // Setup the interface parameters in the socketcan address struct
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_descrp_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("CANInterface: Error while binding to the CAN Socket.");
        } else {
            std::cout << "The Socket Descriptor is: " << socket_descrp_ << std::endl;
        }
    }

    bool CANInterface::sendCANFrame(int can_id, const unsigned char* CANMsg)
    {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        memcpy(frame.data, CANMsg, 8);

        if (write(socket_descrp_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("CANInterface: Error writing to CAN Interface.");
            return false;
        }
        else
        {
            return true;
        }
    }

    bool CANInterface::receiveCANFrame(unsigned char* CANMsg)
    {
        // Listen to all CAN messages. Filter by Motor ID later in the motor driver class.
        struct can_frame frame;

        if (read(socket_descrp_, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("CANInterface: Error Reading Data.");
            return false;
        }
        else
        {
            memcpy(CANMsg, frame.data, frame.can_dlc);
            
            // Save the last received frame to a state interface
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_received_frame_ = frame;
            return true;
        }
    }

    CANInterface::~CANInterface() {

        if (close(socket_descrp_) < 0) {
            perror("CANInterface: Error Closing CAN Socket.");
        }

    }
}