#include "arm.h"
#include <opencv2/imgproc.hpp>
#include "car.h"
#include "freeflying.h"
#include <algorithm>
#include  "object.h"
#include <cmath>

void EndEffector::hold() {
        is_holding = true;
        is_closing_gripper = true; // Reset closing state
        gripper_closed = true; // Assume gripper is closed when holding
        // held_object = object;


        //search all the objects in the world and find if there is one that is close enough to the end effector
}

void Arm::timeTick(float dt)
{
    // One-shot trigger coming from EndEffector::hold()
    if (end_effector.isClosingGripper()) {
        closeGripper();
        end_effector.is_closing_gripper = false; // consume request
    }

    // Auto target based on holding state (keeps it closed while holding, open otherwise)
    if (end_effector.isHolding()) {
        grip_target = grip_min;
    } else {
        grip_target = grip_max;
    }

    // Animate toward target (non-blocking)
    if (grip_anim || std::fabs(grip_target - grip_ratio) > 1e-4f) {
        float delta    = grip_target - grip_ratio;
        float max_step = grip_speed * dt;
        float step     = std::max(-max_step, std::min(delta, max_step));
        grip_ratio    += step;

        if (std::fabs(grip_target - grip_ratio) <= 1e-3f) {
            grip_ratio = grip_target;
            grip_anim  = false;
        }
    }
}

bool Arm::collides(const Point &p)
{
    // Check for collisions with the arm
    for (const Link &link : links) {
        if (link.collides(p)) {
            return true;
        }
    }
    return false;
}

bool Link::collides(const Point &p) const
{
    // Implement collision detection for the link
    // This is a placeholder implementation; replace with actual logic
    return false;
}

void Arm::draw()
{
    // Base pose of the arm (robot * relative mount)
    Pose parent_pose = poseInWorld();
    Point base_coord(parent_pose.x, parent_pose.y);

    // Optional: guard unsupported parent types
    if (auto *car = dynamic_cast<CarRobot *>(parent_robot)) {
        (void)car;
        // Not implemented for CarRobot yet
        // std::cout << "Arm draw on CarRobot not implemented\n";
        return;
    }

    // Current attachment in world coords and pixels
    Point     attachment = base_coord;
    IndexPair i_attach   = world->worldToIndices(attachment);

    // --- Draw links (accumulate absolute angles) ---
    if (!links.empty()) {
        float angle_accumulated = parent_pose.theta;
        for (const Link &link : links) {
            angle_accumulated += link.angle; // absolute w.r.t. parent_pose
            const float end_x = attachment.x + link.length * std::cos(angle_accumulated);
            const float end_y = attachment.y + link.length * std::sin(angle_accumulated);
            const Point end_point(end_x, end_y);
            const IndexPair i_end = world->worldToIndices(end_point);

            cv::line(world->_display_image,
                     cv::Point(i_attach.c, i_attach.r),
                     cv::Point(i_end.c,    i_end.r),
                     cv::Scalar(0, 0, 0), 2);

            cv::circle(world->_display_image,
                       cv::Point(i_end.c, i_end.r),
                       3, cv::Scalar(0, 0, 0), -1);

            attachment = end_point;
            i_attach   = i_end;
        }

        // --- End effector (bar + two fingers) ---
        const Link &last_link = links.back();

        // Orientation of last link at 'attachment'
        // angle_accumulated already equals the last link world angle here.
        float c = std::cos(angle_accumulated), s = std::sin(angle_accumulated);
        Point dir(-s, c);          // perpendicular to link (bar direction)
        Point gripper_dir(c, s);   // along the link (finger direction)

        // Use animated ratio as "bar_length" exactly like your code did
        float bar_length = grip_ratio;                 // 1.0 open, 0.2 closed
        float half_bar   = bar_length * last_link.length;  // your original scaling

        const IndexPair left_tip  = world->worldToIndices(
            Point(attachment.x - dir.x * half_bar,
                  attachment.y - dir.y * half_bar));
        const IndexPair right_tip = world->worldToIndices(
            Point(attachment.x + dir.x * half_bar,
                  attachment.y + dir.y * half_bar));

        // Bar
        cv::line(world->_display_image,
                 cv::Point(left_tip.c,  left_tip.r),
                 cv::Point(right_tip.c, right_tip.r),
                 cv::Scalar(0, 0, 255), 2);

        // Fingers
        const float gripper_length = 0.4f * last_link.length;
        const IndexPair left_grip = world->worldToIndices(
            Point(attachment.x - dir.x * half_bar + gripper_dir.x * gripper_length,
                  attachment.y - dir.y * half_bar + gripper_dir.y * gripper_length));
        const IndexPair right_grip = world->worldToIndices(
            Point(attachment.x + dir.x * half_bar + gripper_dir.x * gripper_length,
                  attachment.y + dir.y * half_bar + gripper_dir.y * gripper_length));

        cv::line(world->_display_image,
                 cv::Point(left_tip.c, left_tip.r),
                 cv::Point(left_grip.c, left_grip.r),
                 cv::Scalar(0, 0, 255), 2);

        cv::line(world->_display_image,
                 cv::Point(right_tip.c, right_tip.r),
                 cv::Point(right_grip.c, right_grip.r),
                 cv::Scalar(0, 0, 255), 2);
    }
}