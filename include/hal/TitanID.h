/**
 * @file TitanID.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief Titan CAN通信ID表
 * @version 0.1
 * @date 2022-09-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef SRC_TITANID_H_
#define SRC_TITANID_H_

#define HG_DEVICE_TYPE 0x2000000
#define HG_MANUFACTURER_ID 0xC0000
#define HG_OFFSET 64
#define HG_DEVICE_ID 42

#define HG_BASE HG_DEVICE_TYPE + HG_MANUFACTURER_ID + HG_DEVICE_ID

// #define HG_GET_ID   HG_BASE
// #define HG_RETURN_ID   HG_BASE + (HG_OFFSET*1)
// #define HG_GET_FIRMWARE_VERSION   HG_BASE + (HG_OFFSET*2)
// #define HG_RETURN_FRIMWARE_VERSION   HG_BASE + (HG_OFFSET*3)
// #define HG_GET_HW_VERSION   HG_BASE + (HG_OFFSET*4)
// #define HG_RETURN_HW_VERSION   HG_BASE + (HG_OFFSET*5)
// #define HG_GET_UNIQUE_ID   HG_BASE + (HG_OFFSET*6)
// #define HG_RETURN_WORD_1   HG_BASE + (HG_OFFSET*7)
// #define HG_RETURN_WORD_2   HG_BASE + (HG_OFFSET*8)
// #define HG_RETURN_WORD_3   HG_BASE + (HG_OFFSET*9)
// #define HG_CONFIGURE_MOTOR   HG_BASE + (HG_OFFSET*10)
// #define HG_CHECK_MOTOR_FREQUENCY   HG_BASE + (HG_OFFSET*11)
// #define HG_RETURN_MOTOR_FREQUENCY   HG_BASE + (HG_OFFSET*12)
// #define HG_DISABLE_MOTOR   HG_BASE + (HG_OFFSET*13)
// #define HG_SET_MOTOR_SPEED   HG_BASE + (HG_OFFSET*14)
// #define HG_GET_CURRENT   HG_BASE + (HG_OFFSET*15)
// #define HG_RETURN_CURRENT   HG_BASE + (HG_OFFSET*16)
// #define HG_GET_ENCODER_TICKS   HG_BASE + (HG_OFFSET*17)
// #define HG_RETURN_ENCODER_TICKS   HG_BASE + (HG_OFFSET*18)
// #define HG_RESET_ENCODER   HG_BASE + (HG_OFFSET*19)
// #define HG_DISABLED_CHECK   HG_BASE + (HG_OFFSET*20)
// #define HG_ENABLED_CHECK   HG_BASE + (HG_OFFSET*21)
// #define HG_MOTOR_STOP_MODE   HG_BASE + (HG_OFFSET*22)
// #define HG_GET_CONTROLLER_TEMP   HG_BASE + (HG_OFFSET*23)
// #define HG_RETURN_CONTROLLER_TEMP   HG_BASE + (HG_OFFSET*24)
// #define HG_GET_RPM   HG_BASE + (HG_OFFSET*25)
// #define HG_RETURN_RPM   HG_BASE + (HG_OFFSET*26)
// #define HG_GET_LIMIT_INPUTS   HG_BASE + (HG_OFFSET*27)
// #define HG_RETURN_LIMIT_INPUTS   HG_BASE + (HG_OFFSET*28)
#define HG_DISABLE_FLAG HG_BASE
#define HG_ENABLE_FLAG HG_BASE + (HG_OFFSET * 1)
#define HG_SET_MOTOR_SPEED HG_BASE + (HG_OFFSET * 2)
#define HG_DISABLE_MOTOR HG_BASE + (HG_OFFSET * 3)
#define HG_GET_TITAN_INFO HG_BASE + (HG_OFFSET * 4)
#define HG_RETURN_TITAN_INFO HG_BASE + (HG_OFFSET * 5)
#define HG_GET_UNIQUE_ID HG_BASE + (HG_OFFSET * 6)
#define HG_RETURN_WORD_1 HG_BASE + (HG_OFFSET * 7)
#define HG_RETURN_WORD_2 HG_BASE + (HG_OFFSET * 8)
#define HG_RETURN_WORD_3 HG_BASE + (HG_OFFSET * 9)
#define HG_CONFIG_MOTOR HG_BASE + (HG_OFFSET * 10)
#define HG_GET_MOTOR_FREQUENCY HG_BASE + (HG_OFFSET * 11)
#define HG_RETURN_MOTOR_FREQUENCY HG_BASE + (HG_OFFSET * 12)
#define HG_RESET_ENCODER HG_BASE + (HG_OFFSET * 13)
#define HG_SET_CURRENT_LIMIT HG_BASE + (HG_OFFSET * 14)
#define HG_MOTOR_STOP_MODE HG_BASE + (HG_OFFSET * 15)
#define HG_SET_TARGET_VELOCITY HG_BASE + (HG_OFFSET * 16)
#define HG_SET_TARGET_DISTANCE HG_BASE + (HG_OFFSET * 17)
#define HG_ENCODER_0 HG_BASE + (HG_OFFSET * 37)
#define HG_ENCODER_1 HG_BASE + (HG_OFFSET * 38)
#define HG_ENCODER_2 HG_BASE + (HG_OFFSET * 39)
#define HG_ENCODER_3 HG_BASE + (HG_OFFSET * 40)
#define HG_RPM_0 HG_BASE + (HG_OFFSET * 41)
#define HG_RPM_1 HG_BASE + (HG_OFFSET * 42)
#define HG_RPM_2 HG_BASE + (HG_OFFSET * 43)
#define HG_RPM_3 HG_BASE + (HG_OFFSET * 44)
#define HG_LIMIT_SWITCH HG_BASE + (HG_OFFSET * 45)
#define HG_CURRENT HG_BASE + (HG_OFFSET * 46)
#define HG_MCU_TEMP HG_BASE + (HG_OFFSET * 47)

#endif  //  SRC_TITANID_H_
