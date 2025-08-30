#pragma once
#include "world.h"
#include "arm.h"

struct FreeFlying: public WorldItem {
  float tv=0, rv=0; // tv: translational velocity, rv: rotational velocity
  float radius;
  int radius_in_pixels;
  Arm* arm = nullptr;          
  // New constructor
  FreeFlying(shared_ptr<World> w_, float radius_=0.3):
  WorldItem(w_.get()),
  radius_in_pixels(radius_*w_->inv_res)
  {}
  
  // Old constructor for compatibility
  FreeFlying(World* world_, float radius_=0.3):
  WorldItem(world_),
  radius(radius_),
  radius_in_pixels(radius_*world_->inv_res)
  {}
  
  void timeTick(float dt) override;
  bool collides(const Point& p,  bool include_arm = true);
  void draw() override;
  bool hasArm() const { return arm != nullptr; }
  bool armCollidesAtPose(const Pose& test_pose);

};

