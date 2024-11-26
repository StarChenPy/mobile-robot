/**
 * @file vmx.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief
 * @version 0.1
 * @date 2023-05-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "hal/vmx.h"

#include <mutex>

#define LOCK_GUARD(Mutex) std::lock_guard<std::mutex> lock(Mutex)

static std::mutex mtx;

namespace VMX {

VMXPi &VMXPI::GetInstance(bool realtime, uint8_t ahrs_update_rate_hz) {
    static VMXPi vmx(realtime, ahrs_update_rate_hz);
    return vmx;
}

// VMXPi& VMXPI::instance(bool realtime, uint8_t ahrs_update_rate_hz) {
//   static VMXPi vmx(realtime, ahrs_update_rate_hz);
//   return vmx;
// }

float GetSystemVoltage() {
    VMXErrorCode errcode;
    float power = 0.0;
    VMXPI::GetInstance().getPower().GetSystemVoltage(power, &errcode);
    return power;
}
std::string getFirmwareVersion() { return VMXPI::GetInstance().getVersion().GetFirmwareVersion(); }
std::string getHALVersion() { return VMXPI::GetInstance().getVersion().GetHALVersion(); }
bool HicurrdioSupportOutput() {

    uint8_t first_hicurrdio_channel_index;
    uint8_t num_hicurrdio_input_channels =
        VMXPI::GetInstance().io.GetNumChannelsByType(VMXChannelType::HiCurrDIO, first_hicurrdio_channel_index);
    bool supports_output = VMXPI::GetInstance().io.ChannelSupportsCapability(first_hicurrdio_channel_index,
                                                                             VMXChannelCapability::DigitalOutput);
    if (supports_output) {
        std::cout << "HiCurrDIO don't supports input" << std::endl;
    } else {
        std::cout << "HiCurrDIO supports input" << std::endl;
    }
    return supports_output;
}
float getAccFullScaleVoltage() {
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    VMXErrorCode errcode;
    float full_scale_voltage;
    //  LOCK_GUARD(mtx);
    if (VMXPI::GetInstance().io.Accumulator_GetFullScaleVoltage(full_scale_voltage, &errcode)) {
        return full_scale_voltage;
    }
    std::cout << "ERROR acquiring Analog Input Voltage." << GetVMXErrorString(errcode) << std::endl;
    return -1;
}

float getRoll() { return VMXPI::GetInstance().getAHRS().GetRoll(); }
float getPitch() { return VMXPI::GetInstance().getAHRS().GetPitch(); }
float getYaw() { return VMXPI::GetInstance().getAHRS().GetYaw(); }
void zeroYaw() { VMXPI::GetInstance().getAHRS().ZeroYaw(); }

float getCompassHeading() { return VMXPI::GetInstance().getAHRS().GetCompassHeading(); }
bool isCalibrating() { return VMXPI::GetInstance().getAHRS().IsCalibrating(); }
bool isConnected() { return VMXPI::GetInstance().getAHRS().IsConnected(); }
double getByteCount() { return VMXPI::GetInstance().getAHRS().GetByteCount(); }
double getUpdateCount() { return VMXPI::GetInstance().getAHRS().GetUpdateCount(); }
long getLastSensorTimestamp() { return VMXPI::GetInstance().getAHRS().GetLastSensorTimestamp(); }
float getWorldLinearAccelX() { return VMXPI::GetInstance().getAHRS().GetWorldLinearAccelX(); }
float getWorldLinearAccelY() { return VMXPI::GetInstance().getAHRS().GetWorldLinearAccelY(); }
float getWorldLinearAccelZ() { return VMXPI::GetInstance().getAHRS().GetWorldLinearAccelZ(); }
bool isMoving() { return VMXPI::GetInstance().getAHRS().IsMoving(); }
bool isRotating() { return VMXPI::GetInstance().getAHRS().IsRotating(); }
float getBarometricPressure() { return VMXPI::GetInstance().getAHRS().GetBarometricPressure(); }
float getAltitude() { return VMXPI::GetInstance().getAHRS().GetAltitude(); }
bool isAltitudeValid() { return VMXPI::GetInstance().getAHRS().IsAltitudeValid(); }
float getFusedHeading() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().GetFusedHeading();
}
bool isMagneticDisturbance() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().IsMagneticDisturbance();
}
bool isMagnetometerCalibrated() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().IsMagnetometerCalibrated();
}
float getQuaternionW() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().GetQuaternionW();
}
float getQuaternionX() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().GetQuaternionX();
}
float getQuaternionY() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().GetQuaternionY();
}

float getQuaternionZ() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    return VMXPI::GetInstance().getAHRS().GetQuaternionZ();
}
void resetDisplacement() {
    //  LOCK_GUARD(mtx);
    //  HGSYS_ASSERT(m_vmx_  != nullptr);
    VMXPI::GetInstance().getAHRS().ResetDisplacement();
}
void updateDisplacement(float accel_x_g, float accel_y_g, int update_rate_hz, bool is_moving) {

    VMXPI::GetInstance().getAHRS().UpdateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
}
float getVelocityX() { return VMXPI::GetInstance().getAHRS().GetVelocityX(); }
float getVelocityY() { return VMXPI::GetInstance().getAHRS().GetVelocityY(); }
float getVelocityZ() { return VMXPI::GetInstance().getAHRS().GetVelocityZ(); }
float getDisplacementX() { return VMXPI::GetInstance().getAHRS().GetDisplacementX(); }
float getDisplacementY() { return VMXPI::GetInstance().getAHRS().GetDisplacementY(); }
float getDisplacementZ() { return VMXPI::GetInstance().getAHRS().GetDisplacementZ(); }
double getAngle() { return VMXPI::GetInstance().getAHRS().GetAngle(); }
double getRate() { return VMXPI::GetInstance().getAHRS().GetRate(); }
void reset() { VMXPI::GetInstance().getAHRS().Reset(); }

float getRawGyroX() { return VMXPI::GetInstance().getAHRS().GetRawGyroX(); }
float getRawGyroY() { return VMXPI::GetInstance().getAHRS().GetRawGyroY(); }
float getRawGyroZ() { return VMXPI::GetInstance().getAHRS().GetRawGyroZ(); }
float getRawAccelX() { return VMXPI::GetInstance().getAHRS().GetRawAccelX(); }
float getRawAccelY() { return VMXPI::GetInstance().getAHRS().GetRawAccelY(); }
float getRawAccelZ() { return VMXPI::GetInstance().getAHRS().GetRawAccelZ(); }
float getRawMagX() { return VMXPI::GetInstance().getAHRS().GetRawMagX(); }
float getRawMagY() { return VMXPI::GetInstance().getAHRS().GetRawMagZ(); }
float getRawMagZ() { return VMXPI::GetInstance().getAHRS().GetRawMagZ(); }
float getPressure() { return VMXPI::GetInstance().getAHRS().GetPressure(); }
float getTempC() { return VMXPI::GetInstance().getAHRS().GetTempC(); }

int getBoardYawAxis() {
    //  LOCK_GUARD(mtx);
    return VMXPI::GetInstance().getAHRS().GetBoardYawAxis().board_axis;
}

bool Channel::Deactivate() {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().io.DeactivateResource(m_resource_handle, &m_errcode);
    if (!rt) {
        std::cout << "Deactivate false" << GetVMXErrorString(m_errcode) << std::endl;
    }
    rt = VMXPI::GetInstance().io.DeallocateResource(m_resource_handle, &m_errcode);
    if (!rt) {
        std::cout << "Deallocate false" << GetVMXErrorString(m_errcode) << std::endl;
    }
    return rt;
}

SingleChannel::SingleChannel() {
    m_info.index = INVALID_VMX_CHANNEL_INDEX;
    m_info.capabilities = NoCapabilities;
}

bool SingleChannel::Activate(const VMXResourceConfig &res_cfg) {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().io.ActivateSinglechannelResource(m_info, &res_cfg, m_resource_handle, &m_errcode);
    if (!rt) {
        std::cout << "Active IO" << m_info.index << " false" << GetVMXErrorString(m_errcode) << std::endl;
    }
    return rt;
}

DualChannel::DualChannel() {
    m_info[0].index = INVALID_VMX_CHANNEL_INDEX;
    m_info[0].capabilities = NoCapabilities;
    m_info[1].index = INVALID_VMX_CHANNEL_INDEX;
    m_info[1].capabilities = NoCapabilities;
}

bool DualChannel::Activate(const VMXResourceConfig &res_cfg) {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().io.ActivateDualchannelResource(m_info[0], m_info[1], &res_cfg, m_resource_handle,
                                                                  &m_errcode);
    return rt;
}

QuadChannel::QuadChannel() {
    m_info[0].index = INVALID_VMX_CHANNEL_INDEX;
    m_info[0].capabilities = NoCapabilities;
    m_info[1].index = INVALID_VMX_CHANNEL_INDEX;
    m_info[1].capabilities = NoCapabilities;
    m_info[2].index = INVALID_VMX_CHANNEL_INDEX;
    m_info[2].capabilities = NoCapabilities;
    m_info[3].index = INVALID_VMX_CHANNEL_INDEX;
    m_info[3].capabilities = NoCapabilities;
}

bool QuadChannel::Activate(const VMXResourceConfig &res_cfg) {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().io.ActivateQuadchannelResource(m_info[0], m_info[1], m_info[2], m_info[3], &res_cfg,
                                                                  m_resource_handle, &m_errcode);
    return rt;
}

ENC::ENC(uint8_t index, EncoderConfig::EncoderEdge edge) {
    m_info[0].index = index * 2;
    m_info[1].index = index * 2 + 1;
    m_info[0].capabilities = VMXChannelCapability::EncoderAInput;
    m_info[1].capabilities = VMXChannelCapability::EncoderBInput;
    m_config_.edge_count = edge;
    Activate(m_config_);
}

int32_t ENC::read() {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().io.Encoder_GetCount(m_resource_handle, m_counter_, &m_errcode);
    if (!rt) {
        std::cout << "ENC:" << (m_info[0].index) / 2 << " read failed" << GetVMXErrorString(m_errcode);
    }
    return m_counter_;
}
bool ENC::reset() {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().io.Encoder_Reset(m_resource_handle, &m_errcode);
    if (!rt) {
        std::cout << "ENC:" << (m_info[0].index) / 2 << " reset failed" << GetVMXErrorString(m_errcode);
    }
    m_counter_ = 0;
    return rt;
}
ENC::ptr ENC::getPtr() {
    try {
        std::shared_ptr<ENC> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<ENC>(this);
    }
}

DI::DI(uint8_t index, DIOConfig::InputMode input_mode) {
    m_info.index = index;
    // m_config.input = true;
    // m_config.inputmode = input_mode;
    m_info.capabilities = VMXChannelCapability::DigitalInput;
    Activate(m_config);
}

bool DI::read() {
    bool rt = VMXPI::GetInstance().io.DIO_Get(m_resource_handle, m_sigal, &m_errcode);
    if (!rt) {
        std::cout << "DI:" << m_info.index << " read voltage failed" << std::endl;
        return false;
    }
    return m_sigal;
}
DI::ptr DI::getPtr() {
    try {
        std::shared_ptr<DI> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<DI>(this);
    }
}

PWM::PWM(uint8_t index, uint32_t frequency, PWMGeneratorConfig::FrameOutputFilter output_filter,
         uint16_t max_duty_cycle_value) {
    m_info.index = index;
    m_info.capabilities = VMXChannelCapability::PWMGeneratorOutput;
    m_config.frameOutputFilter = output_filter;
    m_config.frequency_hz = frequency;
    m_config.maxDutyCycleValue = max_duty_cycle_value;
    Activate(m_config);
}
void PWM::setDutyCycle(double duty_cycle) {
    m_duty_cycle = (uint16_t)(duty_cycle * m_config.maxDutyCycleValue);
    bool rt = VMXPI::GetInstance().io.PWMGenerator_SetDutyCycle(m_resource_handle, 0, m_duty_cycle, &m_errcode);
    if (!rt) {
        std::cout << "PWM:" << m_info.index << " set PWM duty cycle failed" << std::endl;
        std::cout << GetVMXErrorString(m_errcode) << std::endl;
    }
}
PWM::ptr PWM::getPtr() {
    try {
        std::shared_ptr<PWM> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<PWM>(this);
    }
}

CAN::CAN(int32_t period_ms, VMXCAN::VMXCANMode mode) : m_period_ms_(period_ms) {
    //  LOCK_GUARD(mtx);
    VMXPI::GetInstance().can.DisplayMasksAndFilters();
    bool rt = VMXPI::GetInstance().can.SetMode(mode, &m_errcode_) &&
              VMXPI::GetInstance().can.ResetBusBitrate(VMXCAN::CANBusBitrate::CAN_BUS_BITRATE_1MBPS, &m_errcode_);
    if (!rt) {
        std::cout << "VMX CAN"
                  << " init failed." << GetVMXErrorString(m_errcode_) << std::endl;
    }
}
CAN &CAN::GetInstance(int32_t period_ms, VMXCAN::VMXCANMode mode) {
    static CAN can(period_ms, mode);
    return can;
}
void CAN::flushRxFIFO() {
    //  LOCK_GUARD(mtx);
    if (!VMXPI::GetInstance().can.FlushRxFIFO(&m_errcode_)) {
        std::cout << "Flush Rx FIFO"
                  << " fail." << GetVMXErrorString(m_errcode_) << std::endl;
    }
}
void CAN::flushTxFIFO() {
    //  LOCK_GUARD(mtx);
    if (!VMXPI::GetInstance().can.FlushTxFIFO(&m_errcode_)) {
        std::cout << "Flush Tx FIFO"
                  << " fail." << GetVMXErrorString(m_errcode_) << std::endl;
    }
}
bool CAN::setMode(VMXCAN::VMXCANMode mode) {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().can.SetMode(mode, &m_errcode_);
    if (!rt) {
        std::cout << "Set CAN mode"
                  << " fail." << GetVMXErrorString(m_errcode_) << std::endl;
    }
    return rt;
}
std::string CAN::printMsgInfo(VMXCANMessage msg) {
    std::string s = "";
    for (int i = 0; i < msg.dataSize; i++) {
        s += std::to_string(msg.data[i]);
        s += ",";
    }
    return s;
}
bool CAN::sendMessage(uint32_t SendMessageID, std::vector<uint8_t> data, int32_t period_ms) {
    m_period_ms_ = 1;
    uint8_t *msg_data = new uint8_t[8];
    VMXCANMessage msg;
    msg.messageID = SendMessageID;
    for (int i = 0; i < 8; i++) {
        if (i < data.size()) {
            msg_data[i] = data[i];
            continue;
        }
        msg_data[i] = 0;
    }
    msg.dataSize = 8;
    memcpy(msg.data, msg_data, msg.dataSize);
    // {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().can.SendMessage(msg, m_period_ms_, &m_errcode_);
    if (!rt) {
        std::cout << "SendMessageID:" << SendMessageID << " send message fail"
                  << " message info : " << printMsgInfo(msg) << "." << GetVMXErrorString(m_errcode_);
    }
    // }

    delete[] msg_data;
    return rt;
}
bool CAN::sendMessage(uint32_t SendMessageID, uint8_t data[8], int32_t period_ms) {
    m_period_ms_ = period_ms;
    VMXCANMessage msg;
    msg.messageID = SendMessageID;
    msg.dataSize = 8;
    memcpy(msg.data, data, msg.dataSize);
    // {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().can.SendMessage(msg, m_period_ms_, &m_errcode_);
    if (!rt) {
        std::cout << "SendMessageID:" << SendMessageID << " send message fail"
                  << " message info : " << printMsgInfo(msg) << ". " << GetVMXErrorString(m_errcode_) << std::endl;
    }
    // }
    return rt;
}
bool CAN::createCANReceive(std::string receive_stream_name, uint32_t receive_message_id) {
    ReceiveStreamInfo receive_stream_info;
    receive_stream_info.recvice_message_id_ = receive_message_id;
    // {
    //  LOCK_GUARD(mtx);
    bool rt = VMXPI::GetInstance().can.OpenReceiveStream(
        receive_stream_info.can_rx_handles_, receive_stream_info.recvice_message_id_, receive_stream_info.message_mask_,
        receive_stream_info.max_messages_, &m_errcode_);
    if (!rt) {
        std::cout << "Open receive stream:" << receive_stream_info.recvice_message_id_ << " open fail,"
                  << GetVMXErrorString(m_errcode_) << std::endl;
        return false;
    }
    rt = VMXPI::GetInstance().can.EnableReceiveStreamBlackboard(receive_stream_info.can_rx_handles_, true, &m_errcode_);
    if (!rt) {
        std::cout << "Enable receive stream:" << receive_stream_info.recvice_message_id_ << " open fail"
                  << GetVMXErrorString(m_errcode_) << std::endl;
        return false;
    }
    m_receive_stream_info_.insert(std::pair<string, ReceiveStreamInfo>(receive_stream_name, receive_stream_info));
    // }
    return rt;
}
CAN::ReceiveStreamInfo CAN::readReceiveStream(std::string receive_stream_name) {
    ReceiveStreamInfo no_info;
    auto iter = m_receive_stream_info_.find(receive_stream_name);
    if (iter == m_receive_stream_info_.end()) {
        std::cout << "Can't find receive stream:" << receive_stream_name << std::endl;
        return no_info;
    }
    // {
    //  LOCK_GUARD(mtx);
    // bool rt = VMXPI::instance().can.GetBlackboardEntry(
    //     iter->second.can_rx_handles_, iter->second.recvice_message_id_,
    //     iter->second.messages_, iter->second.sys_timestamp_,
    //     iter->second.already_retrieved_, &m_errcode_);
    uint32_t num_msgs_read;
    bool rt = VMXPI::GetInstance().can.ReadReceiveStream(iter->second.can_rx_handles_, &iter->second.messages_, 1,
                                                         num_msgs_read, &m_errcode_);
    // no_info.already_retrieved_ = iter->second.already_retrieved_;
    no_info.already_retrieved_ = num_msgs_read > 0;
    iter->second.already_retrieved_ = no_info.already_retrieved_;
    if (!rt) {

        std::cout << "Read receive stream:" << iter->second.recvice_message_id_ << " failed"
                  << "receive stream is name:" << receive_stream_name << "." << GetVMXErrorString(m_errcode_);
        return no_info;
    }
    if (!iter->second.already_retrieved_) {
        std::cout << "Read receive stream:" << iter->second.recvice_message_id_ << " read nothing"
                  << " receive stream is name:" << receive_stream_name << std::endl;
        return no_info;
    }
    iter->second.data_.clear();
    for (int i = 0; i < 8; i++) {
        iter->second.data_.push_back(iter->second.messages_.data[i]);
    }
    return iter->second;
    // }
}
std::vector<uint8_t> CAN::readReceiveStreamData(std::string receive_stream_name) {
    ReceiveStreamInfo info = CAN::readReceiveStream(receive_stream_name);

    return info.data_;
}

// CAN::ptr CAN::getPtr() {
//   try {
//     std::shared_ptr<CAN> selfPtr = shared_from_this();
//     // std::cout << "C++:" << selfPtr.get()<< std::endl;
//     return selfPtr;
//   } catch(const std::bad_weak_ptr&) {
//     // std::cout << "C++:" << this<< std::endl;
//     return std::shared_ptr<CAN>(this);
//   }

// }

AI::AI(uint8_t index, uint8_t average_bits) {
    m_info.index = index;
    m_config_.SetNumAverageBits(average_bits);
    m_info.capabilities = VMXChannelCapability::AccumulatorInput;
    Activate(m_config_);
}

float AI::read() {
    bool rt = VMXPI::GetInstance().io.Accumulator_GetAverageVoltage(m_resource_handle, m_voltage_, &m_errcode);
    if (!rt) {
        std::cout << "AI:" << m_info.index << " read voltage failed" << std::endl;
        std::cout << GetVMXErrorString(m_errcode) << std::endl;
        return -1.0;
    }
    return m_voltage_;
}
AI::ptr AI::getPtr() {
    try {
        std::shared_ptr<AI> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<AI>(this);
    }
}

VMXIIC::VMXIIC(uint8_t register_address) : m_register_address_(register_address) {
    m_info[0] = VMXChannelInfo(VMXPI::GetInstance().getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SDA),
                               VMXChannelCapability::I2C_SDA);
    m_info[1] = VMXChannelInfo(VMXPI::GetInstance().getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SCL),
                               VMXChannelCapability::I2C_SCL);
    Activate(m_config_);
}
std::vector<uint8_t> VMXIIC::IICTransaction(std::vector<uint8_t> p_send_data, uint16_t receiveSize) {
    uint8_t *p_send_data_ptr = new uint8_t[p_send_data.size()];
    uint8_t *p_rcv_data_ptr = new uint8_t[receiveSize];
    std::vector<uint8_t> p_rcv_data;
    p_rcv_data.clear();
    for (int i = 0; i < p_send_data.size(); i++) {
        p_send_data_ptr[i] = p_send_data[i];
    }
    bool rt = VMXPI::GetInstance().io.I2C_Transaction(m_resource_handle, m_register_address_, p_send_data_ptr,
                                                      p_send_data.size(), p_rcv_data_ptr, receiveSize, &m_errcode);
    m_success_ = rt;
    if (!rt) {
        std::cout << "IIC"
                  << " transaction failed" << std::endl;
        std::cout << GetVMXErrorString(m_errcode) << std::endl;
        return p_rcv_data;
    }
    for (int i = 0; i < receiveSize; i++) {
        p_rcv_data.push_back(p_rcv_data_ptr[i]);
    }
    return p_rcv_data;
}
VMXIIC::ptr VMXIIC::getPtr() {
    try {
        std::shared_ptr<VMXIIC> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<VMXIIC>(this);
    }
}

ADS1115::ADS1115() { m_iic_ = std::make_shared<VMXIIC>(0x48); }
float ADS1115::read(uint8_t index) {

    std::vector<uint8_t> p_send_data;
    std::vector<uint8_t> receive_data;
    p_send_data.push_back(0x01);
    p_send_data.push_back(0xc2 + index * 0x10);
    p_send_data.push_back(0xe3);
    m_iic_->IICTransaction(p_send_data, 2);
    if (!m_iic_->m_success_) {
        return -1.0;
    }
    p_send_data[0] = 0x00;
    receive_data = m_iic_->IICTransaction(p_send_data, 2);
    if (!m_iic_->m_success_) {
        return -1.0;
    }
    m_channel_[index] = (receive_data[0] & 0x00FF) << 8;
    m_channel_[index] += (receive_data[1] & 0x00FF);
    m_channel_[index] /= 6553.5;
    return m_channel_[index];
}
float ADS1115::get(uint8_t index) { return m_channel_[index]; }
ADS1115::ptr ADS1115::getPtr() {
    try {
        std::shared_ptr<ADS1115> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<ADS1115>(this);
    }
}
DO::DO(uint8_t index, DIOConfig::OutputMode output_mode) {
    m_config_.SetOutputMode(output_mode);
    m_info.index = index;
    m_info.capabilities = VMXChannelCapability::DigitalOutput;
    m_config_.input = false;
    Activate(m_config_);
}

void DO::write(bool sigal) {
    bool rt = VMXPI::GetInstance().io.DIO_Set(m_resource_handle, sigal, &m_errcode);
    m_sigal_ = sigal;
    if (!rt) {
        std::cout << "DO:" << m_info.index << " set sigal failed" << std::endl;
        std::cout << GetVMXErrorString(m_errcode) << std::endl;
    }
}

void DO::setPulse(uint32_t pulse_period_microseconds) {
    bool rt = VMXPI::GetInstance().io.DIO_Pulse(m_resource_handle, true, pulse_period_microseconds, &m_errcode);
    m_pulse_period_microseconds_ = pulse_period_microseconds;
    if (!rt) {
        std::cout << "DO:" << m_info.index << " set pulse failed" << std::endl;
        std::cout << GetVMXErrorString(m_errcode) << std::endl;
    }
}
DO::ptr DO::getPtr() {
    try {
        std::shared_ptr<DO> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<DO>(this);
    }
}

Timer::Timer(uint8_t index) {
    m_info.index = index;
    m_info.capabilities = VMXChannelCapability::InputCaptureInput2;
    m_config_.SetSlaveMode(InputCaptureConfig::SLAVEMODE_RESET);
    m_config_.SetSlaveModeTriggerSource(InputCaptureConfig::TRIGGER_DYNAMIC);
    m_config_.SetCaptureChannelSource(InputCaptureConfig::CH1, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
    m_config_.SetCaptureChannelSource(InputCaptureConfig::CH2, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
    if (VMXPI::GetInstance().io.ChannelSupportsCapability(m_info.index, VMXChannelCapability::InputCaptureInput2)) {
        m_config_.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_FALLING);
        m_config_.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_RISING);
    } else {
        m_config_.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_RISING);
        m_config_.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_FALLING);
    }
    m_config_.SetCaptureChannelPrescaler(InputCaptureConfig::CH1, InputCaptureConfig::x1);
    m_config_.SetCaptureChannelPrescaler(InputCaptureConfig::CH2, InputCaptureConfig::x1);
    m_filter_number_ = m_config_.GetClosestCaptureCaptureFilterNumSamples(2);
    m_config_.SetCaptureChannelFilter(InputCaptureConfig::CH1, this->m_filter_number_);
    m_config_.SetCaptureChannelFilter(InputCaptureConfig::CH2, this->m_filter_number_);
    Activate(m_config_);
}
uint32_t Timer::read() {
    bool rt = VMXPI::GetInstance().io.InputCapture_GetChannelCounts(m_resource_handle, m_ch1_count_, m_ch2_count_,
                                                                    &m_errcode);
    if (!rt) {
        std::cout << "Timer:" << m_info.index << " get counter failed" << std::endl;
        return false;
    }
    return m_ch2_count_;
}

Timer::ptr Timer::getPtr() {
    try {
        std::shared_ptr<Timer> selfPtr = shared_from_this();
        // std::cout << "C++:" << selfPtr.get()<< std::endl;
        return selfPtr;
    } catch (const std::bad_weak_ptr &) {
        // std::cout << "C++:" << this<< std::endl;
        return std::shared_ptr<Timer>(this);
    }
}

} //  namespace VMX
