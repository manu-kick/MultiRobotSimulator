#include "robot.h"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>

void Robot::timeTick(float dt) {
  std::cerr << "rv: " << rv << " tv: " << tv << std::endl;
  Pose next_pose = pose * Pose(tv * dt, 0, rv * dt);
  if (!collides(next_pose.translation())) {
    pose = next_pose;
  } else {
    std::cerr << "collision" << std::endl;
    tv = 0;
    rv = 0;
  }
}

bool Robot::collides(const Point& p) {
  IndexPair p0 = world->worldToIndices(p);
  int r2 = radius_in_pixels * radius_in_pixels;

  for (int r = -radius_in_pixels; r <= radius_in_pixels; ++r) {
    for (int c = -radius_in_pixels; c <= radius_in_pixels; ++c) {
      if (r*r + c*c > r2)
        continue;

      IndexPair ip(p0.r + r, p0.c + c);
      if (!world->isInside(ip))
        return true;
      if (world->at(ip) < 127)
        return true;
    }
  }

  return false;
}

void Robot::draw() {
  // Center of robot in pixel coordinates
  IndexPair center_idx = world->worldToIndices(pose.translation());

  // Draw filled circle representing the robot
  cv::circle(
    world->_display_image,
    cv::Point(center_idx.c, center_idx.r),
    radius_in_pixels,
    cv::Scalar(0, 0, 255),
    -1 // filled
  );

  // Draw heading direction as an arrow
  float theta = pose.rotation();
  float arrow_length = radius_in_pixels * 1.5f;
  int tip_x = center_idx.c + std::sin(theta) * arrow_length;
  int tip_y = center_idx.r + std::cos(theta) * arrow_length;

  cv::arrowedLine(
    world->_display_image,
    cv::Point(center_idx.c, center_idx.r),
    cv::Point(tip_x, tip_y),
    cv::Scalar(255, 0, 0), // yellow arrow
    1,
    cv::LINE_AA,
    0,
    0.3
  );
}
