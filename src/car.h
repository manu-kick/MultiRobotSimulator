#include "world.h"


struct CarRobot: public WorldItem{
    // inputs
    float v = 0; // v: velocity
    float phi = 0; // phi: steering angle

    // parameters
    float L; // L: distance between front and rear axles
    float d; // d: distance from the middle point of the axles and the contact point of the wheels

    CarRobot(World* world_, float L_=8, float d_=5):
        WorldItem(world_),
        L(L_),
        d(d_)
    {}

    void timeTick(float dt) override;

    bool collides(const Point& p);
  
    void draw() override;

    void setVelocity(float velocity) {
        float max_velocity = 10.0; // Maximum velocity
        if (velocity < 0) {            v = 0; // Prevent negative velocity
        } else if (velocity > max_velocity) {
            v = max_velocity; // Cap to maximum velocity
        } else {
            v = velocity;
        }   
    }

    void setSteeringAngle(float steering_angle) {
        // Ensure steering angle is within a reasonable range(-1.6, 1.6)
        float max_steering_angle = 1.4;
        if (steering_angle < -max_steering_angle) {
            phi = -max_steering_angle;
        } else if (steering_angle > max_steering_angle) {
            phi = max_steering_angle;
        } else {
            phi = steering_angle;
        }
    }
};
