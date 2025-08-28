#pragma once
#include <vector>
#include "world.h" // for WorldItem
#include "object.h"

using namespace std;

// Forward declaration of Link
struct Link{
    float length; // Length of the link
    float angle;  // Angle of the link in radians
    int type; // Type of the link (e.g., 0="link", 1="end_effector")
    // Add more properties as needed, e.g., mass, inertia, etc.
    Link(float length_=1.0f, float angle_=0.0f, int type_=0) : length(length_), angle(angle_), type(type_) {}
};

struct EndEffector {
    bool is_holding = false; // Whether the end effector is holding an object
    bool gripper_closed = false; // Whether the end effector is closing its gripper
    Object* held_object = nullptr; // Pointer to the held object, if any
    bool is_closing_gripper = false; // Whether the end effector is closing its gripper

    // Add more properties as needed, e.g., position, orientation, etc.
    EndEffector() = default;
    EndEffector(bool is_holding_, Object* held_object_ = nullptr) : is_holding(is_holding_), held_object(held_object_) {}
    
    void hold();

    void release() {
        is_holding = false;
        held_object = nullptr;
        gripper_closed = false;
        // (we'll auto-open in timeTick)
    }
    bool isHolding() const {
        return is_holding;
    }
    bool isClosingGripper() const {
        return is_closing_gripper;
    }

    WorldItem* getHeldObject() const {
        return held_object;
    }
};

struct Arm: public WorldItem {
    // vector of Links
    std::vector<Link> links;
    EndEffector end_effector; // Optional end effector

    // Robot which belongs to this arm
    WorldItem* parent_robot; // which can be a FreeFlying or CarRobot but also a generic WorldItem if the code will be expanded

    // Animation variables
    float grip_ratio  = 0.6f;   // 1.0=open, 0.2=closed (used as your bar_length)
    float grip_target = 0.6f;
    float grip_min    = 0.2f;
    float grip_max    = 0.6f;
    float grip_speed  = 0.2f;   // units/sec (tune)
    bool  grip_anim   = false;

    void closeGripper() { grip_target = grip_min; grip_anim = true; }
    void openGripper()  { grip_target = grip_max; grip_anim = true; }

    // Constructor
    Arm(WorldItem* parent_robot_ = nullptr, const std::vector<Link>& links_ = {}, const EndEffector& end_effector_ = {})
        : WorldItem(parent_robot_),
          links(links_),
          end_effector(end_effector_),
          parent_robot(parent_robot_)
    {}


    void timeTick(float dt);
    bool collides(const Point& p);
    void draw();

};