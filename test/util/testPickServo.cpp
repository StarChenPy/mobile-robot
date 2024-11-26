#include "RobotCfg.h"
#include "params.h"
using namespace std;
using namespace RobotGenius;
class CommandTest : public CommandBase {
 public:
  typedef std::shared_ptr<CommandTest> Ptr;
  CommandTest(std::string name) :m_name(name){}
  void initialize() override {
    std::cout << m_name << " initialize" << std::endl;
  }


  void execute()override {
    double delta = 0.0005;
    int cnt_time = 1000;
    ClampServo_val = CLAMP_SERVO_MIN + (counter % cnt_time) * delta;
    RaiseServo_val = RAISE_SERVO_MIN + (counter % cnt_time) * delta;
    TelescopicServo_val = TELESCOPIC_SERVO_MIN + (counter % cnt_time) * delta;
    
    if(ClampServo_val >= CLAMP_SERVO_MAX) {ClampServo_val = CLAMP_SERVO_MAX;}
    else if(ClampServo_val <= CLAMP_SERVO_MIN) {ClampServo_val = CLAMP_SERVO_MIN;}
    if(RaiseServo_val >= RAISE_SERVO_MAX) {RaiseServo_val = RAISE_SERVO_MAX;}
    else if(RaiseServo_val <= RAISE_SERVO_MIN) {RaiseServo_val = RAISE_SERVO_MIN;}
    if(TelescopicServo_val >= TELESCOPIC_SERVO_MAX) {TelescopicServo_val = TELESCOPIC_SERVO_MAX;}
    else if(TelescopicServo_val <= TELESCOPIC_SERVO_MIN) {TelescopicServo_val = TELESCOPIC_SERVO_MIN;}

    ClampServo->setDutyCycle(ClampServo_val);
    RaiseServo->setDutyCycle(RaiseServo_val);
    TelescopicServo->setDutyCycle(TelescopicServo_val);
    // SeedingServo->setDutyCycle(SEEDLEFT );
    is_finished = !StopLimit->read(); 

    std::cout << "counter = " << counter << std::endl;
    std::cout << "ClampServo_val = " << ClampServo_val << std::endl;
    std::cout << "RaiseServo_val = " << RaiseServo_val << std::endl;
    std::cout << "TelescopicServo_val = " << TelescopicServo_val << std::endl;

    // std::cout << GetCurrentMS() - m_last_time<< std::endl;
    m_last_time =  GetCurrentMS();

    counter++;
  }
  void end() override{
    std::cout <<  m_name << "end" << std::endl;
  }
  bool isFinished() override {
    if(counter > 1000 ) {
      is_finished = true;
    }
    return is_finished;
  }
 private:
  std::string m_name;
  int counter = 0;
  bool is_finished = false;
  uint8_t m_sleep_time;
  int64_t m_last_time;

  double ClampServo_val = CLAMP_SERVO_MIN;
  double RaiseServo_val = RAISE_SERVO_MIN;
  double TelescopicServo_val = TELESCOPIC_SERVO_MIN;

};
int main() {
  // while(1){
  //   // ClampServo->setDutyCycle(0.115);           //0.2是开，0.1是合
  //   // RaiseServo->setDutyCycle(0.05);         //0.22是高，0.05是低
  //   // TelescopicServo->setDutyCycle(0.1);    //0.2是前，0.1是后
  //   sleep(1);
  // }

  Scheduler::GetInstance(1, false).start();
  sleep(1); 
  CommandTest::Ptr command0 = std::make_shared<CommandTest>("TestPickServo");
  
  command0->withTimer(100)->schedule();
  // sleep(1); 
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}

