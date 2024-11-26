#include "RobotCfg.h"

using namespace std;
using namespace RobotGenius;
class CommandTest : public CommandBase {
 public:
  typedef std::shared_ptr<CommandTest> Ptr;
  CommandTest(std::string name) :m_name(name){}
  void initialize() override {
    StopLimit->read();
    std::cout << m_name << " initialize" << std::endl;
  }
  void execute()override {
    LiftLimit->read();
    SeedingLimit->read();
    is_finished = !StopLimit->read();

    std::cout <<  "LiftLimit:" << LiftLimit->get() <<std::endl;
    std::cout <<  "SeedingLimit:" << SeedingLimit->get() <<std::endl;
    std::cout <<  "StopLimit:" << StopLimit->get() <<std::endl;
    std::cout << GetCurrentMS() - m_last_time<< std::endl;
    m_last_time =  GetCurrentMS();
    // sleep(m_sleep_time);
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

};
int main() {
  CommandTest::Ptr command0 = std::make_shared<CommandTest>("TestLimit");
  
  Scheduler::GetInstance(1, false).start();
  sleep(1);
  command0->withTimer(100)->schedule();
  // sleep(1); 
  sleep(3);
  Scheduler::GetInstance().stop();
  return 0;
}
