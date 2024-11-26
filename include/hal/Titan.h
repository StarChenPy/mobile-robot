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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "TitanID.h"

namespace Titan {

class Titan : public std::enable_shared_from_this<Titan> {
  public:
    typedef std::shared_ptr<Titan> ptr;
    bool setEnable();
    bool setDisable();
    uint32_t getTitanWorld1();
    uint32_t getTitanWorld2();
    uint32_t getTitanWorld3();
    std::string getTitanWorld();
    Titan::ptr getPtr();
};

class Motor : public std::enable_shared_from_this<Motor> {
  public:
    typedef std::shared_ptr<Motor> ptr;
    bool setEnable();
    bool setDisable();
    explicit Motor(uint8_t index, uint16_t frequency = 20000);
    bool setSpeedAndDir(uint8_t speed, bool dir0, bool dir1);
    Motor::ptr getPtr();

  private:
    uint8_t m_index_;
};

class TitanQuandLimit : public std::enable_shared_from_this<TitanQuandLimit> {
  public:
    typedef std::shared_ptr<TitanQuandLimit> ptr;
    TitanQuandLimit();
    bool setEnable();
    bool setDisable();
    bool read(uint8_t index);
    bool get(uint8_t index) { return m_sigal[index]; }
    void read();
    TitanQuandLimit::ptr getPtr();

  private:
    uint8_t m_sigal[8];
};

} //  namespace Titan
