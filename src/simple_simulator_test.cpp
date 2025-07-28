#include <cmath>
#include <cstdint>
#include "opencv2/opencv.hpp"
#include <iostream>
#include "simple_geometry.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "car.h"

using namespace std;

int main(int argc, char **argv)
{
  float res = 0.05;
  float delay = 0.1;
  World w(0.05);
  w.loadFromImage(argv[1]);
  // Lidar l(&w);
  // l.pose=Pose(res*w.rows/2, res*w.cols/2, 0);
  // w.addItem(&l);

  // Robot r(&w, 0.2);
  // w.addItem(&r);

  CarRobot c(&w);
  w.addItem(&c);
  Lidar lr(&c);
  w.addItem(&lr);

  c.pose = Pose(25, 20, M_PI / 2);

  int k;
  //  c.v = 0.1; // Example input for car robot
  // c.phi = 0.05; // Example input for steering angle

  while (1)
  {
    w.timeTick(delay);
    w.show();

    k = cv::waitKeyEx(delay * 1000) & 255;

    switch (k)
    {
#ifdef __APPLE__
    case 2:
      c.setSteeringAngle(c.phi + 0.05);
      break; // left arrow
    case 0:
      c.setVelocity(c.v + 0.1);
      break; // up arrow
    case 3:
      c.setSteeringAngle(c.phi - 0.05);
      break; // right arrow
    case 1:
      c.setVelocity(c.v - 0.1);
      break; // down arrow
    case 114: // 'r' reset initial Pose
      c.setPose(Pose(25, 20, M_PI / 2));
      break; // 'r' key for reset
#endif
    case 32:
      c.setVelocity(0);
      c.setSteeringAngle(0);
      break; // spacebar
    case 27:
      return 0;
    default:;
    }
    cerr << "k: " << (int)k << endl;
  }
}
