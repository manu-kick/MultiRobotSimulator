#pragma once
#include "world.h"
#include "simple_geometry.h"
using namespace std;

struct Prehensile_Point {
    string id;
    float value;

    Prehensile_Point(const string& id_ = "", float value_ = 0.0f) : id(id_), value(value_) {}
};

struct Object : public WorldItem {
    Pose pose; // Pose of the object in the world
    std::vector<Prehensile_Point> prehensile_points; // Prehensile points for grasping
    std::vector<Point> goal; // Goal position for the object (4 points for a box)
    // Size size; // Size of the object


    Object(World* world_, const Pose& pose_ = Pose(), const std::vector<Prehensile_Point>& prehensile_points_ = {}, const std::vector<Point>& goal_ = {}) : WorldItem(world_), pose(pose_), prehensile_points(prehensile_points_), goal(goal_) {}
};


struct BoxObject : public Object {
    float width;  // Width of the box
    float height; // Height of the box
    int edge_num = 4; // Number of edges for the box

    BoxObject(World* world_, float width_, float height_, const Pose& pose_ = Pose(), const std::vector<Prehensile_Point>& prehensile_points_ = {}, const std::vector<Point>& goal_ = {}) : Object(world_, pose_, prehensile_points_, goal_), width(width_), height(height_) {}
    void timeTick(float dt) override;
    void draw() override;
    void drawPrehensile(const Prehensile_Point& pp, const vector<cv::Point>& corners_px);
};