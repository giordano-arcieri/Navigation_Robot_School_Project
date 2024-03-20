#include <webots/Robot.hpp>
#include <MyRobot.h>

using namespace webots;

int main(int argc, char **argv)
{

  MyRobot *robot = new MyRobot();

  int timeStep = (int)robot->getBasicTimeStep();

  while (robot->step(timeStep) != -1)
  {
    robot->run();
  };


  delete robot;

  return 0;

}
