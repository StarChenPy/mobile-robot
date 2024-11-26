#include "RobotCfg.h"
#include "system/Robot.h"


class testGyro : public CommandBase {
 public:
  typedef std::shared_ptr<testGyro> Ptr;
  testGyro() {}
  double getGyro() {return m_gyro;}
  void zeroGyro() {zeroYaw();}
  void print(){std::cout << "m_gyro = " << m_gyro << std::endl;}

  void initialize() override ;
  void execute() override;
  void end() override;
  bool isFinished() override;

  int counter = 0;

 private:
  bool is_finished = false;
  int64_t m_last_time;

  double m_gyro = 0;
};

void testGyro::initialize() {
  is_finished = false;
}
void testGyro::execute() {
  m_gyro = VMX::getYaw();
  print();
  counter++;

  int time = RobotGenius::GetCurrentMS();
  int dt = time - m_last_time;
  m_last_time = time;
  std::cout << "testGyro execute dt = " << dt << std::endl;
  // is_finished = Robot::GetInstance().seed(data_);
}
void testGyro::end() {
  // std::cout << "SeedCommand:endl" <<std::endl;
}
bool testGyro::isFinished() {
  if(Robot::GetInstance().getStopsignal()) {
    stopAll();
  }
  if(counter > 600 ) {
    is_finished = true;
  }
  return is_finished || Robot::GetInstance().getStopsignal();
}



int main() {
  testGyro::Ptr command0 = std::make_shared<testGyro>();
  Scheduler::GetInstance(2, false).start();
  sleep(1);
  command0->withTimer(20)->schedule();
  int cnt = 0;
  while(command0->counter < 600){
    cnt++;
    if(cnt > 10000){cnt = 0;}
    if(cnt % 10 == 5){
      command0->zeroGyro();
    }
    std::cout << "-------------------------------------Yaw = " << command0->getGyro() << std::endl;
    sleep(1);
  }
  // sleep(1); 
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
