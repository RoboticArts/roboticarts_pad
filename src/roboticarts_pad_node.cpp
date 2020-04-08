#include <roboticarts_pad/roboticarts_pad.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roboticarts_pad");
  ros::NodeHandle n;

  RoboticartsPad roboticarts_pad(n);
  roboticarts_pad.run();
}