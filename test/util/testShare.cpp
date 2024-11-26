/**
 * @file testShare.cpp
 * @author jiapeng.lin (jiapeng.lin@high-genius,com)
 * @brief 测试共享内存
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <string>
#include "share.h"

using namespace LABVIEW;
/**
 * @brief 测试共享队列数据满了之后是否能够检测并且不再写入
 *
 */
// void testShareQueue() {
//   SharedQueue<int64_t> queue(0x5000);
//   std::cout << "create" << queue.createSharedQueue(10) << std::endl;
//   std::cout << "data size:" << queue.getSharedQueueStatus()->data_size_
//             << " empty:" << queue.isEmpty() << " full:" << queue.isFull()
//             << " status:" << queue.getSharedQueueStatus()->share_queue_status_  <<  std::endl;
//   for (int64_t i = 0; i < queue.getSharedQueueStatus()->data_size_ + 1; i++) {
//     queue.enqueue(i);
//     std::cout << "enqueue had saved: "
//               << queue.getSharedQueueStatus()->data_size_saved_ <<  " is full: " << queue.getSharedQueueStatus()->is_full_
//               << std::endl;
//     sleep(1);
//   }
//   int64_t a[10];
//   queue.dequeue(a);
//   for (size_t i = 0; i < queue.getSharedQueueStatus()->data_size_; i++)
//   {
//     std::cout << a[i] << std::endl;
//   }
//   queue.clear();
//   queue.destroySharedQueue();
// }
/// @brief 
///1.测试共享内存能不能互相打通读取数据：创建两个manager，拥有相同的key，一个写入，一个读取
///2. 测试能否读取最新的的写入列：读取的延后一个循环，
// void testShareQueueManager() {
//   SharedQueueManager<int> manager(0x5000);
  
  
//   ///创建10个队列，每个队列10个
//   manager.createSharedQueues(3, 5);
//   manager.printStatus();
//   SharedQueueManager<int> manager2(0x5000);
//   ///创建10个队列，每个队列10个
//   manager2.createSharedQueues(3, 5);
//   manager2.printStatus();
//   int a[10];
//   for (int i = 0; i < manager.getShareQueueManagerStatus()->sub_queue_size_ * 3 * 3 +1;
//        i++) {
//     manager.write(i);
//     // manager.getQueueWriting()->printStatus();
//     if (i > 5) {
//       if (manager2.read(a)) {
//         for (int j = 0; j < 5; j++) {
//           std::cout << a[j] << std::endl;
//         }
//       }
//     }
//   }

//   manager.destroySharedQueues();
// }

//  测试共享内存数据
struct Person {
  bool sex = false;
  uint32_t age = 18;
  double name = 0.01;
};

// struct Pose {
//   double x;
//   double y;
//   double thate;
// };
// struct PoseCtrl {
//   Pose pose;
//   double v;
// };

typedef ShareMemory<Person> PersonShareMemory;
void testShareMemory() {
  ShareMemory<int> data_int(0x2000);
  int aaa;
  ShareMemory<bool> data_bool(0x2009);
  bool b_ = false;
  ShareMemory<Person> data(0x6000);
  ShareMemory<Person> data2(0x6000);
  Person p;
    Person p2;
  p.sex = false;
  for (int i = 0; i < 10; i++) {
    p.age = i+10;
    p.name = 0.1*i;
    p.sex = i%2==0;
    data.write(p);
    int d;
    data2.read(p2);

    data_int.write(i);
    data_int.read(aaa);
    std::cout<<"int :"<< aaa << std::endl;
    
    data_bool.write(i%2==0);
    data_bool.read(b_);
    std::cout<<"bool :"<< b_ << std::endl;

    std::cout << "name:" << p2.name << " sex:" << p2.sex << " age:" << p2.age << std::endl;
    sleep(1);
  }
}



void testSharedMemoryArray() {
  int len = 360;
  SharedMemoryArray<double> data(0x9000,len);
  double write[len] = {1,2,3,4};
  double read[len];
  for(int i = 0; i < len; i++){
    write[i] = i;
  }
  data.write(write);
  data.read(read);
  for(int i = 0; i < len; i++){
    std::cout << "i = " << read[i] << std::endl;
  }
}

void testSharedMemoryStructArray() {
  int len = 4;
  SharedMemoryArray<Person> data(0x9000,len);
  Person write[len];
  Person read[len];
  for(int i = 0; i < len; i++){
    write[i].age = i + 10;
    write[i].name = i * 0.1;
    write[i].sex = i%2==0;
  }
  data.write(write);
  data.read(read);
  for(int i = 0; i < len; i++){
    std::cout << "age = " << read[i].age << std::endl;
    std::cout << "name = " << read[i].name << std::endl;
    std::cout << "sex = " << read[i].sex << std::endl;
  }
}


// typedef ShareMemory<PoseCtrl> PoseCtrlShareMemory;
// typedef ShareMemory<Pose> PoseShareMemory;
// typedef ShareMemory<uint8_t> Uint8ShareMemory;
// void testCommandShareMemory() {
//   double cnt = -10;
//   ShareMemory<PoseCtrl> ctrl_data(0x1000);
//   ShareMemory<uint8_t> status_data(0x2000);
//   ShareMemory<Pose> odom(0x3000);
//   PoseCtrl pose_ctrl;
//   uint8_t command_status;
//   Pose cur_pose;
//   while(1){
//     status_data.read(command_status);
//     if(command_status == 1){
//       ctrl_data.read(pose_ctrl);
//       std::cout << "ctrl_data x:" << pose_ctrl.pose.x_ << " ctrl_data y:" << pose_ctrl.pose.y_ << " ctrl_data theta:" << pose_ctrl.pose.theta_ << " ctrl_data v:" << pose_ctrl.params.v << std::endl;
//       uint8_t updata_status = COMMEND_WAIT;
//       status_data.write(updata_status);
      
//     }else if(command_status == 2){
//       cnt = cnt + 1;
//     }else if(command_status == 3){
//       uint8_t updata_status = 0;
//       status_data.write(updata_status);
//     }
//     std::cout << "cnt x:" <<cnt<<std::endl;
//     sleep(1);
//     // cnt++;
//     cur_pose = {cnt*10, cnt, cnt*0.1};
//     odom.write(cur_pose);
//     if(cur_pose.y_ > pose_ctrl.pose.y_){
//       uint8_t updata_status = COMMEND_END;
//       status_data.write(updata_status);
//     }

//     if(cnt > 1000) cnt = 0;
//   }
// }


int main() {
  // testShareQueue();
  // testShareQueueManager();
  // testShareMemory();
  // testCommandShareMemory();
  testSharedMemoryArray();
  testSharedMemoryStructArray();
  return 0;
}