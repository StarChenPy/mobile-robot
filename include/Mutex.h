/**
 * @file Mutex.h
 * @author jiapeng.lin (jiapeng.lin@high-genius.com)
 * @brief 锁模块
 * @version 0.1
 * @date 2022-09-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <semaphore.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <atomic>
#include <list>
#include <utility>
#include <string>
#include "Noncopyable.h"

namespace RobotGenius {

class Semaphore : public Noncopyable {
 public:
  explicit Semaphore(uint32_t count = 0);
  ~Semaphore();

  void wait();
  void notify();
 private:
  sem_t m_semaphore_;
};



template <class T>
struct ScopedLockImpl {
 public:
  explicit ScopedLockImpl(T& mutex)
    :m_mutex_(mutex) {
    this->m_mutex_.lock();
    this->m_locked_ = true;
  }

  ~ScopedLockImpl() {
    this->unlock();
  }
  void lock() {
    if (!this->m_locked_) {
      this->m_mutex_.lock();
      this->m_locked_ = true;
    }
  }
  void unlock() {
    if (this->m_locked_) {
      this->m_mutex_.unlock();
      this->m_locked_ = false;
    }
  }

 private:
  T& m_mutex_;
  bool m_locked_;
};


class Mutex : public Noncopyable {
 public:
  typedef ScopedLockImpl<Mutex> Lock;
  Mutex() {
    pthread_mutex_init(&this->m_mutex_, nullptr);
  }
  ~Mutex() {
    pthread_mutex_destroy(&this->m_mutex_);
  }
  void lock() {
    pthread_mutex_lock(&this->m_mutex_);
  }
  void unlock() {
    pthread_mutex_unlock(&this->m_mutex_);
  }

 private:
  pthread_mutex_t m_mutex_;
};

template <class T>
struct ReadScopedLockImpl {
 public:
  explicit ReadScopedLockImpl(T& mutex)
    :m_mutex_(mutex) {
    this->m_mutex_.rdlock();
    this->m_locked_ = true;
  }

  ~ReadScopedLockImpl() {
    this->unlock();
  }
  void lock() {
    if (!this->m_locked_) {
      this->m_mutex_.rdlock();
      this->m_locked_ = true;
    }
  }
  void unlock() {
    if (this->m_locked_) {
      this->m_mutex_.unlock();
      this->m_locked_ = false;
    }
  }

 private:
  T& m_mutex_;
  bool m_locked_;
};


template <class T>
struct WriteScopedLockImpl {
 public:
  explicit WriteScopedLockImpl(T& mutex)
    :m_mutex_(mutex) {
    this->m_mutex_.wrlock();
    this->m_locked_ = true;
  }

  ~WriteScopedLockImpl() {
    this->unlock();
  }
  void lock() {
    if (!this->m_locked_) {
      this->m_mutex_.wrlock();
      this->m_locked_ = true;
    }
  }
  void unlock() {
    if (this->m_locked_) {
      this->m_mutex_.unlock();
      this->m_locked_ = false;
    }
  }

 private:
  T& m_mutex_;
  bool m_locked_;
};

class NullMutex : public Noncopyable {
 public:
  typedef ScopedLockImpl<NullMutex> Lock;
  NullMutex() {}
  ~NullMutex() {}
  void lock() {}
  void unlock() {}
};


class RWMutex :public Noncopyable {
 public:
  typedef ReadScopedLockImpl<RWMutex> ReadLock;
  typedef WriteScopedLockImpl<RWMutex> WriteLock;

  RWMutex() {
    pthread_rwlock_init(&this->m_lock_, nullptr);
  }
  ~RWMutex() {
    pthread_rwlock_destroy(&this->m_lock_);
  }
  void rdlock() {
    pthread_rwlock_rdlock(&this->m_lock_);
  }
  void wrlock() {
    pthread_rwlock_wrlock(&this->m_lock_);
  }
  void unlock() {
    pthread_rwlock_unlock(&this->m_lock_);
  }
 private:
  pthread_rwlock_t m_lock_;
};

class NullRWMutex :public Noncopyable {
 public:
  typedef ReadScopedLockImpl<NullMutex> ReadLock;
  typedef WriteScopedLockImpl<NullMutex> WriteLock;

  NullRWMutex() {}
  ~NullRWMutex() {}

  void rdlock() {}
  void wrlock() {}
  void unlock() {}
};

class Spinlock : public Noncopyable {
 public:
  typedef ScopedLockImpl<Spinlock> Lock;
  Spinlock() {
    pthread_spin_init(&this->m_mutex_, 0);
  }
  ~Spinlock() {
    pthread_spin_destroy(&this->m_mutex_);
  }
  void lock() {
    pthread_spin_lock(&this->m_mutex_);
  }
  void unlock() {
    pthread_spin_unlock(&this->m_mutex_);
  }
 private:
  pthread_spinlock_t m_mutex_;
};

class CASLock :public  Noncopyable {
 public:
  typedef ScopedLockImpl<CASLock> Lock;
  CASLock() {
    this->m_mutex_.clear();
  }
  ~CASLock() {
  }
  void lock() {
    while (std::atomic_flag_test_and_set_explicit(&this->m_mutex_, std::memory_order_acquire)) {}
  }
  void unlock() {
    std::atomic_flag_clear_explicit(&this->m_mutex_, std::memory_order_release);
  }
 private:
  volatile std::atomic_flag m_mutex_;
};



}  //  namespace RobotGenius

