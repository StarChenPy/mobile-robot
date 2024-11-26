/**
 * @file VMXIO.h
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief
 * @version 0.1
 * @date 2023-05-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <vmxpi/VMXPi.h>

namespace VMX {

class VMXPI {
  public:
    static VMXPi &GetInstance(bool realtime = true, uint8_t ahrs_update_rate_hz = 100);
};
/**
 * @brief 获取错误信息
 *
 * @return char*
//  */
// const char* GetVMXError(VMXErrorCode errcode) {
//   return GetVMXErrorString(errcode);
// }
/**
 * @brief 获取系统供电电压
 *
 * @return float
 */
float GetSystemVoltage();
std::string getFirmwareVersion();
std::string getHALVersion();

/**
 * @brief Hicurrdio是否支持输出
 *
 * @return true
 * @return false
 */
bool HicurrdioSupportOutput();
/**
 * @brief 获取模拟通道最高可采集电压值
 *
 * @return float
 */
float getAccFullScaleVoltage();

///  板载IMU相关函数

/**
 * @brief 获取Roll角
 *
 * @return float
 */
float getRoll();
/**
 * @brief 获取Pitch角
 *
 * @return float
 */
float getPitch();
/**
 * @brief 获取Yaw角
 *
 * @return float
 */
float getYaw();
/**
 * @brief Yaw清零
 *
 */
void zeroYaw();
float getCompassHeading();
bool isCalibrating();
bool isConnected();
double getByteCount();
double getUpdateCount();
long getLastSensorTimestamp();
float getWorldLinearAccelX();
float getWorldLinearAccelY();
float getWorldLinearAccelZ();
bool isMoving();
bool isRotating();
float getBarometricPressure();
float getAltitude();
bool isAltitudeValid();
float getFusedHeading();
bool isMagneticDisturbance();
bool isMagnetometerCalibrated();
float getQuaternionW();
float getQuaternionX();
float getQuaternionY();
float getQuaternionZ();
void resetDisplacement();
void updateDisplacement(float accel_x_g, float accel_y_g, int update_rate_hz, bool is_moving);
float getVelocityX();
float getVelocityY();
float getVelocityZ();
float getDisplacementX();
float getDisplacementY();
float getDisplacementZ();
double getAngle();
double getRate();
void reset();
float getRawGyroX();
float getRawGyroY();
float getRawGyroZ();
float getRawAccelX();
float getRawAccelY();
float getRawAccelZ();
float getRawMagX();
float getRawMagY();
float getRawMagZ();
float getPressure();
float getTempC();

int getBoardYawAxis();

/**
 * @brief Channel类,所有通道的基类
 *
 */
class Channel {
  public:
    typedef std::shared_ptr<Channel> ptr;
    /**
     * @brief 构造函数
     *
     */
    Channel() {}
    /**
     * @brief 析构函数,释放引用
     *
     */
    virtual ~Channel() {}
    /**
     * @brief 重置
     *
     * @return true
     * @return false
     */
    // bool Reset();
    /**
     * @brief 使能
     *
     * @return true
     * @return false
     */
    // virtual bool setEnable() = 0;

  protected:
    /**
     * @brief 使能
     *
     * @return true
     * @return false
     */
    virtual bool Activate(const VMXResourceConfig &res_cfg) = 0;

  public:
    /**
     * @brief 失能并且注销
     *
     * @return true
     * @return false
     */
    bool Deactivate();

  protected:
    //  vmx错误编号
    VMXErrorCode m_errcode;
    //  引用
    VMXResourceHandle m_resource_handle;
};

/**
 * @brief 单一通道类
 *
 */
class SingleChannel : public Channel {
  public:
    typedef std::shared_ptr<SingleChannel> ptr;
    /**
     * @brief 默认构造函数
     *
     */
    SingleChannel();
    /**
     * @brief 析构函数
     *
     */
    virtual ~SingleChannel() {}
    bool Activate(const VMXResourceConfig &res_cfg) override;

  protected:
    ///  通道信息
    VMXChannelInfo m_info;
};

/**
 * @brief 双通道配置
 *
 */
class DualChannel : public Channel {
  public:
    typedef std::shared_ptr<DualChannel> ptr;
    DualChannel();
    ~DualChannel() {}
    bool Activate(const VMXResourceConfig &res_cfg) override;

  protected:
    VMXChannelInfo m_info[2];
};

class QuadChannel : public Channel {
  public:
    typedef std::shared_ptr<QuadChannel> ptr;
    QuadChannel();
    ~QuadChannel() {}
    bool Activate(const VMXResourceConfig &res_cfg) override;

  protected:
    VMXChannelInfo m_info[4];
};

class AI : public SingleChannel, std::enable_shared_from_this<AI> {
  public:
    typedef std::shared_ptr<AI> ptr;
    explicit AI(uint8_t index, uint8_t average_bits = 1);
    ~AI() {}
    /**
     * @brief 获取模拟量
     *
     */
    float read();
    float get() { return m_voltage_; }
    AI::ptr getPtr();

  private:
    ///  模拟通道配置
    AccumulatorConfig m_config_;
    float m_voltage_;
};

/**
 * @brief 编码器类
 *
 */
class ENC : public DualChannel, std::enable_shared_from_this<ENC> {
  public:
    typedef std::shared_ptr<ENC> ptr;
    explicit ENC(uint8_t index, EncoderConfig::EncoderEdge edge = EncoderConfig::EncoderEdge::x4);
    ~ENC() {}

    int32_t read();
    int32_t get() { return m_counter_; }
    bool reset();
    ENC::ptr getPtr();

  private:
    //  编码器索引
    uint8_t m_index_;
    int32_t m_counter_ = 0;  //<  编码器计数值
    EncoderConfig m_config_; //  编码器配置
};

class DI : public SingleChannel, std::enable_shared_from_this<DI> {
  public:
    typedef std::shared_ptr<DI> ptr;
    /**
     * @brief 构造函数
     *
     * @param index
     */
    explicit DI(uint8_t index, DIOConfig::InputMode input_mode = DIOConfig::InputMode::NONE);
    /**
     * @brief 析构函数
     *
     */
    ~DI() {}
    bool read();
    bool get() { return m_sigal; }
    DI::ptr getPtr();

  private:
    //  DIO配置
    DIOConfig m_config;
    //  信号
    bool m_sigal;
};

/**
 * @brief 普通IO口设置为PWM
 *
 */
class PWM : public SingleChannel, std::enable_shared_from_this<PWM> {
  public:
    typedef std::shared_ptr<PWM> ptr;
    /**
     * @brief 构造函数
     *
     * @param index
     */
    explicit PWM(uint8_t index, uint32_t frequency = 50,
                 PWMGeneratorConfig::FrameOutputFilter output_filter = PWMGeneratorConfig::FrameOutputFilter::NONE,
                 uint16_t max_duty_cycle_value = 255);
    /**
     * @brief 析构函数
     *
     */
    ~PWM() {}
    void setDutyCycle(double duty_cycle);
    PWM::ptr getPtr();

  private:
    //  PWM占空比
    uint16_t m_duty_cycle = 0;
    //  PWM配置
    PWMGeneratorConfig m_config;
};

class VMXIIC : public DualChannel, std::enable_shared_from_this<VMXIIC> {
  public:
    typedef std::shared_ptr<VMXIIC> ptr;
    explicit VMXIIC(uint8_t register_address);
    ~VMXIIC() {}
    std::vector<uint8_t> IICTransaction(std::vector<uint8_t> p_send_data, uint16_t receiveSize);
    VMXIIC::ptr getPtr();

  private:
    I2CConfig m_config_;
    uint8_t m_register_address_;

  public:
    bool m_success_;
};

class ADS1115 : public std::enable_shared_from_this<ADS1115> {
  public:
    typedef std::shared_ptr<ADS1115> ptr;
    ADS1115();
    ~ADS1115() {}
    float read(uint8_t index);
    float get(uint8_t index);
    ADS1115::ptr getPtr();

  private:
    VMXIIC::ptr m_iic_;
    float m_channel_[4];
};

class DO : public SingleChannel, std::enable_shared_from_this<DO> {
  public:
    typedef std::shared_ptr<DO> ptr;
    explicit DO(uint8_t index, DIOConfig::OutputMode output_mode = DIOConfig::OutputMode::PUSHPULL);
    ~DO() {}

    void write(bool sigal);
    bool get() { return m_sigal_; }

    void setPulse(uint32_t pulse_period_microseconds = 15);
    DO::ptr getPtr();

  private:
    //  DIO配置
    DIOConfig m_config_;
    //  信号
    bool m_sigal_;
    uint32_t m_pulse_period_microseconds_ = 0;
};

class Timer : public SingleChannel, std::enable_shared_from_this<Timer> {
  public:
    typedef std::shared_ptr<Timer> ptr;
    explicit Timer(uint8_t index);
    ~Timer() {}
    uint32_t read();
    uint32_t get() { return m_ch2_count_; }
    Timer::ptr getPtr();

  private:
    //  计数器1的数值
    uint32_t m_ch1_count_;
    //  计数器2的数值
    uint32_t m_ch2_count_;
    //  中断配置
    InputCaptureConfig m_config_;
    uint8_t m_filter_number_;
};

class CAN : public std::enable_shared_from_this<CAN> {
  public:
    typedef std::shared_ptr<CAN> ptr;
    static CAN &GetInstance(int32_t period_ms = 0, VMXCAN::VMXCANMode mode = VMXCAN::VMXCANMode::VMXCAN_NORMAL);
    // typedef HGSys::Mutex MutexType;

    struct ReceiveStreamInfo {
        // CAN接收流引用
        VMXCANReceiveStreamHandle can_rx_handles_;
        uint32_t recvice_message_id_ = 0;
        // 含时间戳消息
        VMXCANTimestampedMessage messages_;
        // 读取的最大消息数
        uint32_t messages_to_read_ = 1;
        // 读取到的消息数量
        uint32_t messages_read_;
        // 消息掩码
        uint32_t message_mask_ = 0x1FFFFFFF;
        // 接收到的消息的最大数量
        uint32_t max_messages_ = 100;
        uint64_t sys_timestamp_;
        bool already_retrieved_;
        bool rt_;
        std::vector<uint8_t> data_;
    };
    explicit CAN(int32_t period_ms = 0, VMXCAN::VMXCANMode mode = VMXCAN::VMXCANMode::VMXCAN_NORMAL);
    ~CAN() {}

    /* Flush Rx/Tx fifo not necessary if invoking reset above. */
    void flushRxFIFO();
    void flushTxFIFO();
    bool setMode(VMXCAN::VMXCANMode mode);
    std::string printMsgInfo(VMXCANMessage msg);
    bool sendMessage(uint32_t SendMessageID, std::vector<uint8_t> data, int32_t period_ms = 0);
    bool sendMessage(uint32_t SendMessageID, uint8_t *data, int32_t period_ms = 0);
    bool createCANReceive(std::string receive_stream_name, uint32_t receive_message_id);
    ReceiveStreamInfo readReceiveStream(std::string receive_stream_name);
    std::vector<uint8_t> readReceiveStreamData(std::string receive_stream_name);

  private:
    // CAN总线状态
    VMXCANBusStatus m_can_bus_status_;
    // CAN总线消息
    VMXCANMessage m_msg_;
    // 循环发送消息时间
    int32_t m_period_ms_ = 0;
    std::map<std::string, ReceiveStreamInfo> m_receive_stream_info_;
    VMXErrorCode m_errcode_;
};

// typedef Singleton<CAN> VMXCANMgr;

} // namespace VMX