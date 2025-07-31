#include <cmath>
#include <cstdint>
#include "opencv2/opencv.hpp"
#include <iostream>
#include "simple_geometry.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"

using namespace std;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
        return -1;
    }
    float res = 0.05;
    float delay = 0.1;
    World w(0.05);
    w.loadFromImage(argv[1]);

    Lidar l(&w);
    l.pose = Pose(res * w.rows / 2, res * w.cols / 2, 0);
    w.addItem(&l);

    Robot r(&w, 0.2);
    w.addItem(&r);
    r.pose = Pose(10, 10, M_PI / 2);



    while (1)
    {
        w.timeTick(delay);
        w.show();
        // k=cv::waitKeyEx(delay*1000)&255;


    }

    return 0;
}