#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace CAN_interface {
    class CANInterface {
    public:
        CANInterface(const char* socketName);
        ~CANInterface();
        
        bool sendCANFrame(int can_id, const unsigned char* CANMsg);
        bool receiveCANFrame(unsigned char* CANMsg);

    private:
        int socket_descrp_;
    };
}

#endif // CAN_INTERFACE_HPP
