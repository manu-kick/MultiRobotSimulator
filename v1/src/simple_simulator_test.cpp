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

  Robot r(&w, 2);
  w.addItem(&r);
  r.pose = Pose(25, 20, M_PI / 2);
  Lidar lr(&r);
  w.addItem(&lr);

  // CarRobot c(&w);
  // w.addItem(&c);
  // Lidar lr(&c);
  // w.addItem(&lr);
  // c.pose = Pose(25, 20, M_PI / 2);

  int k;
  cout << "Press ESC to exit, spacebar to stop, 'r' to reset pose." << endl;
  

  while (1)
  {
    w.timeTick(delay);
    w.show();

    k = cv::waitKeyEx(delay * 1000) & 255;

    switch (k)
    {
#ifdef __APPLE__
    case 2:
      // c.setSteeringAngle(c.phi + 0.05);
      r.rv += 0.1; // left arrow
      break; // left arrow
    case 0:
      // c.setVelocity(c.v + 0.1);
      r.tv += 0.1; // up arrow
      break; // up arrow
    case 3:
      // c.setSteeringAngle(c.phi - 0.05);
      r.rv -= 0.1; // right arrow
      break; // right arrow
    case 1:
      // c.setVelocity(c.v - 0.1);
      r.tv -= 0.1; // down arrow
      break; // down arrow
    case 114: // 'r' reset initial Pose
      // c.setPose(Pose(25, 20, M_PI / 2));
      r.pose = Pose(25, 20, M_PI / 2);
      break; // 'r' key for reset
#endif
    case 32:
      // c.setVelocity(0);
      // c.setSteeringAngle(0);
      r.tv = 0;
      r.rv = 0;
      break; // spacebar
    case 27:
      return 0;
    default:;
    }
    cerr << "k: " << (int)k << endl;
  }
}
