#pragma once
#include "world.h"
#include "simple_geometry.h"
#include "utils.h"
using namespace std;

struct Prehensile_Point {
    string id;
    float value;
    float grasp_area_size;  // lato del quadrato attorno al punto prensile
    Point grasp_px;         // coordinate in pixel del punto prensile

    Prehensile_Point(const string& id_ = "", float value_ = 0.0f, float grasp_area_size_ = 40.0f) : id(id_), value(value_), grasp_area_size(grasp_area_size_), grasp_px(-1, -1) {}
};

struct Object : public WorldItem {
    Pose pose; // Pose of the object in the world
    Color primaryColor = Color(0,0,0); // Color for visualization
    Color secondaryColor = Color(0,0,0);

    std::vector<Prehensile_Point> prehensile_points; // Prehensile points for grasping
    std::vector<Point> goal; // Goal position for the object (4 points for a box)
    // Size size; // Size of the object
    bool locked = false;

    Object(World* world_, const Pose& pose_ = Pose(), const std::vector<Prehensile_Point>& prehensile_points_ = {}, const std::vector<Point>& goal_ = {}) 
        : WorldItem(world_), pose(pose_), prehensile_points(prehensile_points_), goal(goal_) {
            primaryColor = Color::generateRandomColor();
            secondaryColor = generateSecondaryColor(primaryColor);
        }
    bool isInsideGraspArea(const cv::Point& effector_px, std::string& grasp_id);
    bool isInsideGoalArea(const Point& p);
    bool collides();
};

struct BoxObject : public Object {
    float width;  // Width of the box
    float height; // Height of the box
    int edge_num = 4; // Number of edges for the box

    BoxObject(World* world_, float width_, float height_, const Pose& pose_ = Pose(), const std::vector<Prehensile_Point>& prehensile_points_ = {}, const std::vector<Point>& goal_ = {}) : Object(world_, pose_, prehensile_points_, goal_), width(width_), height(height_) {}
    void timeTick(float dt) override;
    bool collides();
    void draw() override;
    void drawPrehensile(Prehensile_Point& pp, const vector<cv::Point>& corners_px);
    bool isPointCollidingWithBoxObject(const Point &p);
};