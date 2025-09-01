#include "mrsim/ros_bridge.h"

RosBridge::RosBridge(World &w, std::vector<RobotHandle> &robots)
    : rclcpp::Node("mrsim_bridge"), world_(w), robots_(robots)
{
    pr_.reserve(robots_.size());
    for (auto &rh : robots_)
    {
        // 1) Create a slot inside pr_ and get a stable pointer to it
        pr_.emplace_back();
        PerRobot *slot = &pr_.back();
        slot->h = &rh;
        const std::string ns = "/robot/" + rh.id;

        // 2) Capture the stable pointer (by value) in lambdas
        slot->sub_cmd = create_subscription<geometry_msgs::msg::Twist>(
            ns + "/cmd_vel", 10,
            [this, slot](geometry_msgs::msg::Twist::SharedPtr m)
            { onCmdVel(m, *slot); });

        slot->sub_joints_cmd = create_subscription<sensor_msgs::msg::JointState>(
            ns + "/arm/joint_cmd", 10,
            [this, slot](sensor_msgs::msg::JointState::SharedPtr m)
            { onJointCmd(m, *slot); });

        slot->sub_gripper = create_subscription<std_msgs::msg::Bool>(
            ns + "/arm/gripper", 10,
            [this, slot](std_msgs::msg::Bool::SharedPtr m)
            { onGripper(m, *slot); });

        slot->pub_joint_states = create_publisher<sensor_msgs::msg::JointState>(
            ns + "/joint_states", 10);
    }
}

void RosBridge::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg, PerRobot &pr)
{
    RCLCPP_DEBUG(this->get_logger(), "cmd_vel for %s: vx=%.3f wz=%.3f",
                 pr.h ? pr.h->id.c_str() : "<null>",
                 msg->linear.x, msg->angular.z);
    if (auto *ff = dynamic_cast<FreeFlying *>(pr.h->ptr))
    {
        ff->tv = static_cast<float>(msg->linear.x);
        ff->rv = static_cast<float>(msg->angular.z);
    }
    else if (auto *car = dynamic_cast<CarRobot *>(pr.h->ptr))
    {
        car->setVelocity(static_cast<float>(msg->linear.x));
        car->setSteeringAngle(static_cast<float>(msg->angular.z));
    }
}

void RosBridge::onJointCmd(const sensor_msgs::msg::JointState::SharedPtr msg, PerRobot &pr)
{
    RCLCPP_DEBUG(this->get_logger(), "joint_cmd for %s: N=%zu",
                 pr.h ? pr.h->id.c_str() : "<null>",
                 msg->position.size());
    Arm *arm = nullptr;
    if (auto *ff = dynamic_cast<FreeFlying *>(pr.h->ptr))
    {
        if (ff->hasArm())
            arm = ff->arm;
    }
    else if (auto *car = dynamic_cast<CarRobot *>(pr.h->ptr))
    {
        if (car->hasArm())
            arm = car->arm;
    }
    if (!arm)
        return;

    const size_t N = std::min(msg->position.size(), arm->links.size());
    for (size_t i = 0; i < N; ++i)
    {
        const float old = arm->links[i].angle;
        arm->links[i].angle = static_cast<float>(msg->position[i]);
        if (arm->collides())
            arm->links[i].angle = old;
    }
}

void RosBridge::onGripper(const std_msgs::msg::Bool::SharedPtr msg, PerRobot &pr)
{
    RCLCPP_DEBUG(this->get_logger(), "gripper for %s: %s",
                 pr.h ? pr.h->id.c_str() : "<null>",
                 msg->data ? "HOLD" : "RELEASE");
    Arm *arm = nullptr;
    if (auto *ff = dynamic_cast<FreeFlying *>(pr.h->ptr))
    {
        if (ff->hasArm())
            arm = ff->arm;
    }
    else if (auto *car = dynamic_cast<CarRobot *>(pr.h->ptr))
    {
        if (car->hasArm())
            arm = car->arm;
    }
    if (!arm)
        return;

    if (msg->data)
        arm->end_effector.hold();
    else
        arm->doRelease();
}

void RosBridge::publishJointStates()
{
    for (auto &pr : pr_)
    {
        Arm *arm = nullptr;
        if (auto *ff = dynamic_cast<FreeFlying *>(pr.h->ptr))
        {
            if (ff->hasArm())
                arm = ff->arm;
        }
        else if (auto *car = dynamic_cast<CarRobot *>(pr.h->ptr))
        {
            if (car->hasArm())
                arm = car->arm;
        }
        if (!arm)
            continue;

        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name.resize(arm->links.size());
        js.position.resize(arm->links.size());
        for (size_t i = 0; i < arm->links.size(); ++i)
        {
            js.name[i] = "joint_" + std::to_string(i);
            js.position[i] = arm->links[i].angle;
        }
        pr.pub_joint_states->publish(js);
    }
}

void RosBridge::spinOnce()
{
    rclcpp::spin_some(shared_from_this());
    publishJointStates();
}
