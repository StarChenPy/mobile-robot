/**
 * @file thread.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 线程模块
 * @version 0.1
 * @date 2022-09-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#pragma once
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <atomic>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include "Mutex.h"

namespace RobotGenius {

class Thread {
 public:
  typedef std::shared_ptr<Thread> ptr_;
  /**
   * @brief 构造函数
   * 
   * @note 如果name为空，则默认命名为UNKNOW，创建线程后需要等待县城正常启动信号，否则线程创建会一直堵塞
   * 
   * @param cb 线程执行函数
   * @param name 线程名称
   */
  Thread(std::function<void()> cb, const std::string& name);
  /**
   * @brief 析构函数
   * 
   */
  ~Thread();
  /**
   * @brief 获取线程id
   * 
   * @return pid_t 
   */
  pid_t getId() const {return this->m_id_;}
  /**
   * @brief 获取线程名称
   * 
   * @return const std::string& 
   */
  const std::string& getName() const {return this->m_name_;}
  /**
   * @brief 等待线程执行结束
   * 
   */
  void join();
  /**
   * @brief 获取当前的线程指针
   * 
   * @return Thread* 
   */
  static Thread* GetThis();
  /**
   * @brief 获取当前线程名称
   * 
   * @return const std::string& 
   */
  static const std::string& GetName();
  /**
   * @brief 设置当前线程名称
   * 
   * @param name 
   */
  static void SetName(const std::string& name);

 private:
  Thread(const Thread&) = delete;
  Thread(const Thread&&) = delete;
  // Thread& operetor=(const Thread&) = delete;

  /**
   * @brief 线程执行函数
   * 
   * @param arg 
   * @return void* 
   */
  static void* run(void* arg);

 private:
  ///  线程id
  pid_t m_id_ = -1;
  pthread_t m_thread_ = 0;
  ///  线程执行函数
  std::function<void()> m_cb_;
  ///  线程名称
  std::string m_name_;
  ///  信号量
  Semaphore m_semaphore_;
};



}  //  namespace RobotGenius



