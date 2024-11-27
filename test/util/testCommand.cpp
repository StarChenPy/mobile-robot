#include "RobotCfg.h"

using namespace std;
using namespace RobotGenius;
class CommandTest : public CommandBase {
 public:
  typedef std::shared_ptr<CommandTest> Ptr;
  CommandTest(std::string name) :m_name(name){}
  CommandTest(std::string name, int cnt) :m_name(name), cnt_limit(cnt){}
  void initialize() override {
    StopLimit->read();
    std::cout << m_name << " initialize" << std::endl;
  }
  void execute()override {
    is_finished = !StopLimit->read();

    std::cout <<  m_name  << " execute" << " time: " << GetCurrentMS() - m_last_time <<std::endl;
    // std::cout << GetCurrentMS() - m_last_time<< std::endl;
    m_last_time =  GetCurrentMS();
    // sleep(m_sleep_time);
    counter++;
  }
  void end() override{
    std::cout <<  m_name << " end!!!!!!!!!!!!!!!!!!!!" << std::endl;
  }
  bool isFinished() override {
    if(counter > cnt_limit ) {
      is_finished = true;
    }
    return is_finished;
  }
 private:
  std::string m_name;
  int counter = 0;
  int cnt_limit = 1000;
  bool is_finished = false;
  uint8_t m_sleep_time;
  int64_t m_last_time;

};

Command::Ptr CarDG(std::string name){
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    std::make_shared<CommandTest>("vision",3)->withTimer(2000)
  );
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    std::make_shared<CommandTest>("Tracking")->withTimer(1000)
  );
  DG->setDeadlineCommand(sequential);
  return DG;
}


Command::Ptr TrackingIdentifyPick(std::string pose, std::string fruit, int column){
  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
  auto identifyAction = std::make_shared<CommandTest>("identifyAction",2)->withTimer(1000);
  auto trackingAction = std::make_shared<CommandTest>("trackingAction",5)->withTimer(1000);

  auto servoAction = std::make_shared<CommandTest>("servoAction",1)->withTimer(1000);
  auto pickAction = std::make_shared<CommandTest>("pickAction",1)->withTimer(1000);

  RG->AddCommands(identifyAction, trackingAction);
  sequential->AddCommands(
    servoAction,
    RG,
    pickAction
  );
  // DG->AddCommands(sequential);
  // DG->setDeadlineCommand(trackingAction);
  // DG->disableShceduleDeadlineCommand();
  // return DG;
  return sequential;
}


// class CommandT2 : public CommandBase {
//   void initialize() override {
//   }
//   void execute()override {
//     ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
//     SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
//     sequential->AddCommands(
//       std::make_shared<CommandTest>("vision",2)->withTimer(1000)
//     );
//     DG->AddCommands(
//       std::make_shared<CommandTest>("Tracking")->withTimer(1000)
//     );
//     DG->setDeadlineCommand(sequential);
//     DG->m_parent = getPtr();
//     counter++;
//     std::cout << "1111111111111111111111111111111111111111111111111111111111111111111111111" << std::endl;
//     m_state = Command::State::HOLDON;
//     DG->schedule();
//   }
//   void end() override{
//   }
//   bool isFinished() override {
//     return counter > 3;
//   }
//  int counter = 0;
// };



class DGHoldonCommand : public CommandBase {
  public:
  typedef std::shared_ptr<CommandTest> Ptr;
  // DGHoldonCommand(Command::Ptr dg, std::string name) : m_DG(dg), m_name(name){}
  // DGHoldonCommand(Command::Ptr dg, std::string name, int cnt) :m_DG(dg), m_name(name), cnt_limit(cnt){}
  DGHoldonCommand(std::string pose, std::string fruit, int col) : EndPose(pose), PickFruit(fruit), column(col){}
  void initialize() override {
  }
  void execute()override {
    counter++;
    std::cout << "1111111111111111111111111111111111111111111111111111111111111111111111111" << std::endl;
    std::cout << "isFinisheddec:" << isFinisheddec() << std::endl;
    if ((m_state != Command::State::FINISHED || m_state != Command::State::STOP)&& !isFinisheddec() ) {
      m_state = Command::State::HOLDON;
      // auto car = CarDG(m_name);
      car = TrackingIdentifyPick(EndPose, PickFruit, column);
      car->m_parent = getPtr();
      car->schedule();
    // m_DG->schedule();
    }
    
  }
  void end() override{
    car->cancel();
    std::cout << "DGHoldonCommand end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  }
  bool isFinished() override {
    
    return counter > 100;
  }


 private:
  std::string m_name;
  int counter = 0;
  int cnt_limit = 1000;
  bool is_finished = false;
  uint8_t m_sleep_time;
  int64_t m_last_time;

  Command::Ptr car;
  std::string EndPose;
  std::string PickFruit;
  int column;
};



int main() {
  Scheduler::GetInstance(4, false).start();
  ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
  RG->AddCommands(
    std::make_shared<DGHoldonCommand>("Pose(240,0,0)", "fruit", 1),
    std::make_shared<CommandTest>("Reach Point", 2)->withTimer(4000)
  );
  SequentialCommandGroup::Ptr sequential = std::make_shared<SequentialCommandGroup>();
  sequential->AddCommands(
    std::make_shared<CommandTest>("test start", 1)->withTimer(500),
    // std::make_shared<DGHoldonCommand>(CarDG("vision Tracking"), "vision Tracking"),
    // std::make_shared<DGHoldonCommand>("Pose(240,0,0)", "fruit", 1),
    RG,
    std::make_shared<CommandTest>("test end", 1)->withTimer(500)
  );

  ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
  DG->AddCommands(
    // std::make_shared<CommandTest>("Left")->withTimer(200),
    // std::make_shared<CommandTest>("Right")->withTimer(200),
    std::make_shared<CommandTest>("Odom")->withTimer(20000)
  );
  DG->setDeadlineCommand(sequential);
  DG->schedule();



  // ParallelRaceGroup::Ptr r = std::make_shared<ParallelRaceGroup>();
  // r->AddCommands(
  //   std::make_shared<CommandTest>("Pose(240,0,0)", 5)->withTimer(500),
  //   std::make_shared<CommandTest>("Reach Point", 2)->withTimer(1000),
  //   std::make_shared<CommandTest>("test", 10)->withTimer(300)
  // );
  // r->schedule();

  sleep(1);
  Scheduler::GetInstance().stop();
  return 0;
}




// #include <iostream>
// #include <memory>
// #include "robotgenius/RobotGenius.h"

// using namespace std;
// using namespace RobotGenius;
// class CommandTest : public CommandBase {
//  public:
//   typedef std::shared_ptr<CommandTest> Ptr;
//   CommandTest(std::string name, uint8_t sleep_time = 0) :m_name(name), m_sleep_time (sleep_time){}
//   void initialize() override {
//     std::cout << m_name << " initialize" << std::endl;
//   }
//   void execute()override {

//     std::cout <<  m_name << "execute:" << counter << " " << GetCurrentMS() - m_last_time<< std::endl;
//     m_last_time =  GetCurrentMS();
//     sleep(m_sleep_time);
//     counter++;
//   }
//   void end() override{
//     std::cout <<  m_name << "end" << std::endl;
//   }
//   void cancel() override{
//     is_finished = true;
//   }
//   bool isFinished() override {
//     if(counter > 10) {
//       is_finished = true;
//     }
//     return is_finished;
//   }
//  private:
//   std::string m_name;
//   int counter = 0;
//   bool is_finished = false;
//   uint8_t m_sleep_time;
//   int64_t m_last_time;

// };

// int main() {
//   CommandTest::Ptr command0 = std::make_shared<CommandTest>("command0");
//   CommandTest::Ptr command1 = std::make_shared<CommandTest>("command1", 1);
//   CommandTest::Ptr command2 = std::make_shared<CommandTest>("command2", 2);
//   CommandTest::Ptr command3 = std::make_shared<CommandTest>("command3", 1);
//   ParallelDeadlineGroup::Ptr DG = std::make_shared<ParallelDeadlineGroup>();
//   SequentialCommandGroup::Ptr S = std::make_shared<SequentialCommandGroup>();
//   // CG->AddCommands(test1->withTimer(20), test2->withTimer(10), test3->withTimer(40));
//   ParallelRaceGroup::Ptr RG = std::make_shared<ParallelRaceGroup>();
//   RG->AddCommands(command2, command1);
//   S->AddCommands(command0, RG, command3);
//   DG->AddCommands(S);
//   DG->setDeadlineCommand(command1);
//   DG->disableShceduleDeadlineCommand();
//   Scheduler::GetInstance(5, false).start();
//   DG->schedule();
//   sleep(10);
//   Scheduler::GetInstance().stop();
//   return 0;
// }

