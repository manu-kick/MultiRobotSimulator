#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "mrsim/arm.h"
#include "mrsim/car.h"
#include "mrsim/freeflying.h"
#include "mrsim/world.h"
#include "mrsim/object.h"
#include "mrsim/utils.h"

// ------------------- util: angolo normalizzato -------------------
static inline float atan2f_safe(float y, float x)
{
    return std::atan2(y, x);
}
static inline float normalizeAngle(float a)
{
    while (a > M_PI)
        a -= 2.f * M_PI;
    while (a < -M_PI)
        a += 2.f * M_PI;
    return a;
}
// void blockRobotMotion(WorldItem *robot)
// {
//     if (!robot)
//         return;
//     if (auto *car = dynamic_cast<CarRobot *>(robot))
//     {
//         car->v = 0;
//         car->phi = 0;
//         // throw cout << "\t\t !!!!Cannot move object, collision detected, robot stopped!!!!" <<endl;
//     }
//     else if (auto *ff = dynamic_cast<FreeFlying *>(robot))
//     {
//         ff->tv = 0;
//         ff->rv = 0;
//     }
// }

// ------------------- Arm::dock -------------------
bool Arm::dock(Object *&out_obj, std::string &out_grasp_id)
{
    out_obj = nullptr;
    out_grasp_id.clear();
    if (!world)
        return false;

    // Scan the items of the world searching for objects (ok if number of objs relatively small)
    for (int i = 0; i < world->num_items; ++i)
    {
        auto *obj = dynamic_cast<Object *>(world->items[i]);
        if (!obj)
            continue;

        // Check whether the EE position is within the grasping area
        if (obj->isInsideGraspArea(cv::Point(end_effector.pixel_pos.c, end_effector.pixel_pos.r), out_grasp_id) && !obj->locked)
        {
            out_obj = obj;
            return true;
        }

        if (obj->locked)
            cout << "\t\t !!!!Cannot dock, obj locked!!!!" << endl;
    }
    return false;
}

// ------------------- Arm::alignLastLinkTo -------------------
void Arm::alignLastLinkTo(const Point &target_world)
{
    if (links.empty())
        return;

    // calcola base e orientamento accumulato fino al link N-1
    Pose parent_pose = poseInWorld();

    cout << "\t-Parent pose x=" << parent_pose.x << " y=" << parent_pose.y << endl;

    Point joint = Point(parent_pose.x, parent_pose.y);
    float theta = parent_pose.theta;

    for (size_t i = 0; i + 1 < links.size(); ++i)
    {
        theta += links[i].angle;
        joint = Point(joint.x + links[i].length * std::cos(theta), joint.y + links[i].length * std::sin(theta));
    }

    IndexPair joint_px = world->worldToIndices(joint);

    // direzione dal giunto al target
    cout << "\t\t-Joint x=" << joint.x << "    y=" << joint.y << " =======> x=" << joint_px.c << " y=" << joint_px.r << endl;
    cout << "\t\t-Target world x=>" << target_world.x << " y=>" << target_world.y << endl;

    float dx = target_world.x - joint_px.c;
    float dy = target_world.y - joint_px.r;

    float desired_abs = atan2f_safe(dx, dy); // angolo assoluto desiderato del last link
    float rel_angle = normalizeAngle(desired_abs - theta);
    cout << "\t\t-dx =" << dx << " dy=" << dy << " Desired angle = " << desired_abs << " normalized=" << rel_angle << endl;

    // setta l'angolo relativo del last link
    links.back().angle = rel_angle;

    // lunghezza necessaria per raggiungere il target
    float dist = std::sqrt(dx * dx + dy * dy);
    cout << "\t\t-dist=" << dist * world->resolution << endl;

    // consenti un "piccolo allungamento" oltre l’originale (es. +10%)
    // float orig = original_lengths.empty() ? links.back().length : original_lengths.back();
    // float max_len = orig * 1.1f; // 10% stretch
    // links.back().length = std::min(dist, max_len);

    // new: reach target exactly — we restore the original on release anyway
    links.back().length = dist * world->resolution;
}

// ------------------- Arm::doRelease -------------------
void Arm::doRelease()
{
    if (links.empty())
        return;
    // ripristina lunghezza originale dell’ultimo link
    if (!original_lengths.empty())
    {
        links.back().length = original_lengths.back();
    }
    end_effector.release();
    has_hold_offset = false;
    hold_offset_local = Point(0, 0);
}

// ------------------- Arm::timeTick -------------------
void Arm::timeTick(float dt)
{
    // ---- Small local helpers (no class API changes) ------------------------
    struct EEState
    {
        Point pos_world{0, 0};
        float theta{0.f};
        IndexPair px{0, 0};
    };

    auto readEEState = [&]() -> EEState
    {
        EEState s;
        computeEndEffectorPose(s.pos_world, s.theta);
        if (world)
            s.px = world->worldToIndices(s.pos_world);
        return s;
    };

    auto objectCenterPx = [&](const Object *obj) -> IndexPair
    {
        return world->worldToIndices(obj->pose.translation());
    };

    auto findGraspPx = [&](const Object *obj, const std::string &id) -> Point
    {
        for (const auto &pp : obj->prehensile_points)
            if (pp.id == id)
                return pp.grasp_px;
        return Point(-1, -1);
    };

    // Given EE pixel position, object theta, object center px and grasp px,
    // compute the NEW object center pixels so that EE = center + R(θ)*offset.
    auto computeNewObjectCenterPx = [&](const IndexPair &ee_px,
                                        float obj_theta,
                                        const IndexPair &obj_center_px,
                                        const Point &grasp_px) -> Point
    {
        // offset in px from center to grasp (column=x, row=y in image indices)
        IndexPair offset_px(grasp_px.y - obj_center_px.r,
                            grasp_px.x - obj_center_px.c);
        float c = std::cos(obj_theta), s = std::sin(obj_theta);
        // center = EE - R(θ)*offset
        return Point(
            ee_px.c - (c * offset_px.c - s * offset_px.r),
            ee_px.r - (s * offset_px.c + c * offset_px.r));
    };
    // ------------------------------------------------------------------------

    // 0) Refresh EE telemetry once (based on current links)
    EEState ee = readEEState();
    end_effector.position = ee.pos_world;
    end_effector.orientation = ee.theta;
    if (world)
        end_effector.pixel_pos = ee.px;

    // 1) Dock request handling (raised by EndEffector::hold())
    if (end_effector.dock_request && !end_effector.isHolding())
    {
        Object *obj = nullptr;
        std::string grasp_id;
        if (dock(obj, grasp_id) && obj)
        {
            // Locate the chosen grasp point (in px)
            Point grasp_px = findGraspPx(obj, grasp_id);
            end_effector.saved_grasp_id = grasp_id; // store for later use while holding

            if (grasp_px.x >= 0 && grasp_px.y >= 0)
            {
                // Aim last link to the grasp point (in px)
                alignLastLinkTo(grasp_px);

                // Recompute EE (FK) once after changing last link
                ee = readEEState();

                // Current object center (px)
                IndexPair obj_center_px = objectCenterPx(obj);

                // Compute new object center so that EE coincides with grasp
                Point new_center_px = computeNewObjectCenterPx(
                    ee.px, obj->pose.theta, obj_center_px, grasp_px);

                // Update object pose (world units)
                obj->pose.y = new_center_px.x * world->resolution;
                obj->pose.x = new_center_px.y * world->resolution;

                // Holding state
                end_effector.is_holding = true;
                end_effector.held_object = obj;
                end_effector.gripper_closed = true;
            }
        }
        // Consume request regardless of outcome
        end_effector.dock_request = false;
    }

    // 2) If holding an object, make it follow the EE (keep the same local offset)
    if (end_effector.isHolding() && end_effector.held_object)
    {
        Object *obj = end_effector.held_object;

        // Which grasp point? Reuse the one saved at docking
        std::string grasp_id = end_effector.saved_grasp_id;
        if (grasp_id.empty() && !obj->prehensile_points.empty())
            grasp_id = obj->prehensile_points.front().id;

        IndexPair obj_center_px = objectCenterPx(obj);
        Point grasp_px = findGraspPx(obj, grasp_id);
        if (grasp_px.x >= 0 && grasp_px.y >= 0)
        {
            // Compute candidate new center from EE
            Point new_center_px = computeNewObjectCenterPx(
                ee.px, obj->pose.theta, obj_center_px, grasp_px);

            // Save old pose before moving
            Pose old_pose = obj->pose;

            // Assign candidate pose (world coords)
            obj->pose.y = new_center_px.x * world->resolution;
            obj->pose.x = new_center_px.y * world->resolution;
            // If you also want orientation to follow EE:
            // obj->pose.theta = end_effector.orientation;

            // --- Check collision for the object itself ---
            if (obj->collides())
            {
                // Restore old pose if collision detected
                obj->pose = old_pose;

                // Stop the robot/base to avoid pushing further
                // blockRobotMotion(parent_robot);
            }
        }
    }

    // (Grip animation etc. can stay here if you have it elsewhere)
}

// Optional: if not already declared elsewhere.
static inline float clampf(float v, float a, float b) { return std::max(a, std::min(v, b)); }

bool pointInCollisionWithItems(const Point &p, WorldItem *item)
{
    if (auto *car = dynamic_cast<CarRobot *>(item))
    {
        const float wheel_len = car->L / 4.0f;
        const float wheel_w = car->d / 4.0f;
        const std::vector<Point> carHull = carHullWorld(
            car->pose, car->L, car->d, wheel_len, wheel_w);
        if (pointInConvexPolygon(carHull, p))
            return true;
    }
    else if (auto *ff = dynamic_cast<FreeFlying *>(item))
    {
        const float r = ff->radius_in_pixels * ff->world->resolution;
        const Point c = ff->pose.translation();
        if (dist2(p, c) < r * r)
            return true;
    }
    auto *obj = dynamic_cast<BoxObject *>(item);
    if (obj)
    {
        // Simple box collision: check if point is inside object's bounding box
        if (obj->isPointCollidingWithBoxObject(p))
            return true;
    }
    return false;
}

bool pointCollides(World *world, const Point &p, const WorldItem *parent_robot)
{
    if (!world)
        return false;

    // Map (pixel) checks
    IndexPair ip = world->worldToIndices(p);
    if (!world->isInside(ip))        // outside map => collision
        return true;
    if (world->at(ip) < 127)         // in an obstacle => collision
        return true;

    // Robot-vs-robot: compare p with every other robot's shape
    for (WorldItem *other : world->items)
    {
        if (other == parent_robot)   // skip my parent (the base carrying this arm)
            continue;
        if (pointInCollisionWithItems(p, other))
            return true;
    }

    

    return false; // clear
}
// ===========================================================================

bool Arm::collides()
{
    if (!world)
    {
        return false;
    }


    // LINK Collision check (sample along each link segment)
   auto segmentCollides = [&](const Point &a, const Point &b) -> bool
    {
        const float dx = b.x - a.x, dy = b.y - a.y;
        const float len = std::sqrt(dx * dx + dy * dy);

        // Sample roughly every half pixel in world units (never zero)
        const float step = std::max(float(world->resolution) * 0.5f, 1e-4f);
        const int   N    = std::max(1, int(std::ceil(len / step)));

        for (int i = 1; i <= N; ++i)    // start at 1 to avoid rechecking the exact joint pixel
        {
            const float t = float(i) / float(N);
            Point p(a.x + t * dx, a.y + t * dy);
            if (pointCollides(world, p, parent_robot))
                return true;
        }
        return false;
    };

    // Walk the chain: base (parent pose) -> joint_1 -> ... -> end-effector
    const Pose base = poseInWorld();
    Point joint(base.x, base.y);
    float theta = base.theta;

    // Optionally also verify the base joint point isn't inside obstacles/robots
    // (uncomment if you want to flag this via the arm instead of the base)
    // if (pointCollides(world, joint, parent_robot)) return true;

    for (const Link &link : links)
    {
        theta += link.angle;
        const Point next(
            joint.x + link.length * std::cos(theta),
            joint.y + link.length * std::sin(theta)
        );

        if (segmentCollides(joint, next))
        {
            // std::cout << "Collision: link segment colliding\n";
            return true;
        }
        joint = next; // advance to the next joint
    }

    // Final EE tip check (mostly redundant with the last segment sample, but cheap & explicit)
    if (pointCollides(world, joint, parent_robot))
    {
        // std::cout << "Collision: EE colliding\n";
        return true;
    }


    return false;
}

void Arm::computeEndEffectorPose(Point &out_pos, float &out_theta) const
{
    Pose parent_pose = poseInWorld();
    Point att(parent_pose.x, parent_pose.y);
    float a = parent_pose.theta;

    for (const Link &link : links)
    {
        a += link.angle;
        att = Point(att.x + link.length * std::cos(a),
                    att.y + link.length * std::sin(a));
    }
    out_pos = att;
    out_theta = a;
}

void Arm::draw()
{
    // Pose base dell’arm (robot * mount relativo)
    Pose parent_pose = poseInWorld();
    Point base_coord(parent_pose.x, parent_pose.y);

    if (auto *car = dynamic_cast<CarRobot *>(parent_robot))
    {
        (void)car;
        return;
    }

    Point attachment = base_coord;
    IndexPair i_attach = world->worldToIndices(attachment);

    float angle_accumulated = parent_pose.theta;

    if (!links.empty())
    {
        for (const Link &link : links)
        {
            angle_accumulated += link.angle;
            const float end_x = attachment.x + link.length * std::cos(angle_accumulated);
            const float end_y = attachment.y + link.length * std::sin(angle_accumulated);
            const Point end_point(end_x, end_y);
            const IndexPair i_end = world->worldToIndices(end_point);

            cv::line(world->_display_image,
                     cv::Point(i_attach.c, i_attach.r),
                     cv::Point(i_end.c, i_end.r),
                     cv::Scalar(0, 0, 0), 1);

            cv::circle(world->_display_image,
                       cv::Point(i_end.c, i_end.r),
                       2, cv::Scalar(0, 0, 0), -1);

            attachment = end_point;
            i_attach = i_end;
        }

        // End effector: semplice cerchio
        cv::circle(world->_display_image,
                   cv::Point(i_attach.c, i_attach.r),
                   6, cv::Scalar(0, 0, 0), -1);
        // ✅ Correct telemetry:
        end_effector.position = attachment;           // WORLD coords (meters / map units)
        end_effector.pixel_pos = i_attach;            // PIXEL indices (r,c)
        end_effector.orientation = angle_accumulated; // absolute EE angle
    }
}
