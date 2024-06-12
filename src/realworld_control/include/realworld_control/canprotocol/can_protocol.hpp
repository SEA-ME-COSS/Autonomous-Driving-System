#pragma once

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

#include <cstdlib> 

#include "dbcppp/Network.h"

class CANSender {
public:
    CANSender(const std::string& dbc_file, const std::string& can_interface);
    ~CANSender();

    void sendSpeedMessage(double speed, uint32_t target_message_id);
    void sendSteerMessage(double steer, uint32_t target_message_id);

private:
    int sock;
    std::unique_ptr<dbcppp::INetwork> net;
    std::unordered_map<uint32_t, const dbcppp::IMessage*> messages;

    void setupVcanInterface(const std::string& can_interface);
};
