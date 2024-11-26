/**
 * @file Titan.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief Titan单元控制
 * @version 0.1
 * @date 2022-09-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "hal/Titan.h"

#include <unistd.h>

#include "hal/TitanID.h"
#include "hal/vmx.h"

namespace Titan {

bool Titan::setEnable() {
    std::vector<uint8_t> data;
    return VMX::CAN::GetInstance().sendMessage(HG_ENABLE_FLAG, data, 5);
}

bool Titan::setDisable() {
    std::vector<uint8_t> data;
    return VMX::CAN::GetInstance().sendMessage(HG_DISABLE_FLAG, data);
}

uint32_t Titan::getTitanWorld1() {
    // setEnable();
    VMX::CAN::GetInstance().createCANReceive("word1", HG_RETURN_WORD_1);
    VMX::CAN::ReceiveStreamInfo info;
    sleep(1);
    info = VMX::CAN::GetInstance().readReceiveStream("word1");
    int32_t word1 = 0;
    word1 = info.messages_.data[0] + (info.messages_.data[1] << 8) + (info.messages_.data[2] << 16) +
            (info.messages_.data[3] << 24);
    // printf("%08X - ", word1);
    return word1;
}
uint32_t Titan::getTitanWorld2() {
    // setEnable();
    VMX::CAN::GetInstance().createCANReceive("word1", HG_RETURN_WORD_2);
    VMX::CAN::ReceiveStreamInfo info;
    sleep(1);
    info = VMX::CAN::GetInstance().readReceiveStream("word2");
    int32_t word2 = 0;
    word2 = info.messages_.data[0] + (info.messages_.data[1] << 8) + (info.messages_.data[2] << 16) +
            (info.messages_.data[3] << 24);
    // printf("%08X - ", word1);
    return word2;
}
uint32_t Titan::getTitanWorld3() {
    // setEnable();
    VMX::CAN::GetInstance().createCANReceive("word1", HG_RETURN_WORD_3);
    VMX::CAN::ReceiveStreamInfo info;
    sleep(1);
    info = VMX::CAN::GetInstance().readReceiveStream("word3");
    int32_t word3 = 0;
    word3 = info.messages_.data[0] + (info.messages_.data[1] << 8) + (info.messages_.data[2] << 16) +
            (info.messages_.data[3] << 24);
    // printf("%08X - ", word1);
    return word3;
}
std::string Titan::getTitanWorld() {
    return std::to_string(getTitanWorld1()) + std::to_string(getTitanWorld2()) + std::to_string(getTitanWorld3());
}
Titan::ptr Titan::getPtr() {
    try {
        std::shared_ptr<Titan> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<Titan>(this);
    }
}
Motor::Motor(uint8_t index, uint16_t frequency) : m_index_(index) {
    setEnable();
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = m_index_;
    data[1] = (frequency & 0x00FF);
    data[2] = (frequency & 0xFF00) >> 8;
    uint32_t request_message_id = HG_CONFIG_MOTOR;
    if (!VMX::CAN::GetInstance().sendMessage(request_message_id, data)) {
        std::cout << "Active Titan motor" << m_index_ << " failed" << std::endl;
    }
}
bool Motor::setSpeedAndDir(uint8_t speed, bool dir0, bool dir1) {
    // setEnable();
    uint32_t request_message_id = HG_SET_MOTOR_SPEED;
    VMXCANMessage msg;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    data[0] = m_index_;
    data[1] = speed;
    data[2] = (uint8_t)dir0;
    data[3] = (uint8_t)dir1;
    if (!VMX::CAN::GetInstance().sendMessage(request_message_id, data)) {
        std::cout << "Set Titan motor" << m_index_ << " speed and dir failed" << std::endl;
        return false;
    }
    return true;
}
Motor::ptr Motor::getPtr() {
    try {
        std::shared_ptr<Motor> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<Motor>(this);
    }
}

bool Motor::setEnable() {
    std::vector<uint8_t> data;
    return VMX::CAN::GetInstance().sendMessage(HG_ENABLE_FLAG, data, 5);
}

bool Motor::setDisable() {
    std::vector<uint8_t> data;
    return VMX::CAN::GetInstance().sendMessage(HG_DISABLE_FLAG, data);
}

TitanQuandLimit::TitanQuandLimit() { VMX::CAN::GetInstance().createCANReceive("TitanQuandLimit", HG_LIMIT_SWITCH); }

bool TitanQuandLimit::read(uint8_t index) {
    VMX::CAN::ReceiveStreamInfo info;
    info = VMX::CAN::GetInstance().readReceiveStream("TitanQuandLimit");
    return static_cast<bool>(info.messages_.data[index]);
}
void TitanQuandLimit::read() {
    VMX::CAN::ReceiveStreamInfo info;
    info = VMX::CAN::GetInstance().readReceiveStream("TitanQuandLimit");
    if (!info.already_retrieved_) {
        // std::cout << "sssssss" << std::endl;
        return;
    }
    // std::cout << "dddd" << std::endl;
    m_sigal[0] = info.messages_.data[0];
    m_sigal[1] = info.messages_.data[1];
    m_sigal[2] = info.messages_.data[2];
    m_sigal[3] = info.messages_.data[3];
    m_sigal[4] = info.messages_.data[4];
    m_sigal[5] = info.messages_.data[5];
    m_sigal[6] = info.messages_.data[6];
    m_sigal[7] = info.messages_.data[7];
}
TitanQuandLimit::ptr TitanQuandLimit::getPtr() {
    try {
        std::shared_ptr<TitanQuandLimit> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<TitanQuandLimit>(this);
    }
}
bool TitanQuandLimit::setEnable() {
    std::vector<uint8_t> data;
    return VMX::CAN::GetInstance().sendMessage(HG_ENABLE_FLAG, data, 5);
}
bool TitanQuandLimit::setDisable() {
    std::vector<uint8_t> data;
    return VMX::CAN::GetInstance().sendMessage(HG_DISABLE_FLAG, data);
}

} //  namespace Titan