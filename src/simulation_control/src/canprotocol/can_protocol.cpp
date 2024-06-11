#include "canprotocol/can_protocol.hpp"

CANSender::CANSender(const std::string& dbc_file, const std::string& can_interface) {
    setupVcanInterface(can_interface);

    std::ifstream idbc(dbc_file);
    this->net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    if (!this->net) {
        throw std::runtime_error("Failed to parse DBC file.");
    }

    for (const auto& msg : this->net->Messages()) {
        std::cout << "Msg ID : " << msg.Id() << std::endl;
        this->messages[msg.Id()] = &msg;
    }

    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0) {
        throw std::runtime_error("Error while opening socket.");
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface.c_str());
    ioctl(sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(this->sock, (struct sockaddr *)&addr, sizeof(addr));

    std::cout << "CAN init Success!" << std::endl;
}

CANSender::~CANSender() {}

void CANSender::setupVcanInterface(const std::string& can_interface) {
    std::string command = "sudo modprobe vcan && sudo ip link add dev " + can_interface + " type vcan && sudo ip link set up " + can_interface;

    std::cout << command << std::endl;
    int ret = system(command.c_str());
    if (ret != 0) {
        // throw std::runtime_error("Error setting up vcan interface.");
    }
}

void CANSender::sendSpeedMessage(double speed, uint32_t target_message_id) {
    // std::unordered_map<uint64_t, const dbcppp::IMessage*>::iterator iter = messages.find(target_message_id);
    // if (iter == messages.end()) {
    //     throw std::runtime_error("Message with ID 255 not found in DBC.");
    // }

    // const dbcppp::IMessage& msg = *(iter->second);
    // std::vector<uint8_t> frame_data(8, 0);
    // for (const dbcppp::ISignal& sig : msg.Signals()) {
    //     if (sig.Name() == "Speed") {
    //         double raw_value = sig.PhysToRaw(speed);
    //         sig.Encode(raw_value, frame_data.data());
    //     }
    // }

    struct can_frame frame;
    frame.can_id = target_message_id;
    frame.can_dlc = 8;
    
    std::memset(frame.data, 0, sizeof(frame.data));
    std::memcpy(frame.data, &speed, sizeof(speed));

    ssize_t nbytes = write(sock, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        throw std::runtime_error("Error while sending speed frame.");
    } 
    else {
        std::cout << "Message sent successfully: Speed = " << speed << std::endl;
    }
}

void CANSender::sendSteerMessage(double steer, uint32_t target_message_id) {
    // std::unordered_map<uint64_t, const dbcppp::IMessage*>::iterator iter = messages.find(target_message_id);
    // if (iter == messages.end()) {
    //     throw std::runtime_error("Message with ID 255 not found in DBC.");
    // }

    // const dbcppp::IMessage& msg = *(iter->second);
    // std::vector<uint8_t> frame_data(8, 0);
    // for (const dbcppp::ISignal& sig : msg.Signals()) {
    //     if (sig.Name() == "Speed") {
    //         double raw_value = sig.PhysToRaw(speed);
    //         sig.Encode(raw_value, frame_data.data());
    //     }
    // }

    struct can_frame frame;
    frame.can_id = target_message_id;
    frame.can_dlc = 8;
    
    std::memset(frame.data, 0, sizeof(frame.data));
    std::memcpy(frame.data, &steer, sizeof(steer));

    ssize_t nbytes = write(sock, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        throw std::runtime_error("Error while sending steer frame.");
    } 
    else {
        std::cout << "Message sent successfully: Steer = " << steer << std::endl;
    }
}

