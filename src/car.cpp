#include "car.h"
#include <cmath>
#include <opencv2/imgproc.hpp> // for cv::line and cv::rectangle

void CarRobot::timeTick(float dt)
{
    cout << "phi: " << phi << " v: " << v << endl;

    // Compute change in orientation
    float dtheta = (v / L) * std::tan(phi) * dt;

    // Move forward along car’s heading (x-axis in robot frame)
    // This respects non-holonomic constraint — no lateral slip
    Pose next_pose = pose * Pose(v * dt, 0.0f, dtheta);

    if (!collides(next_pose.translation()))
    {
        pose = next_pose;
    }
    else
    {
        cerr << "collision" << endl;
        v = 0;
        phi = 0;
    }
}

bool CarRobot::collides(const Point &p)
{
    // Car pose (rear axle center)
    float theta = pose.rotation();
    Point rear = p;
    Point dir(std::cos(theta), std::sin(theta));
    Point front (rear.x + dir.x * L, rear.y + dir.y * L);
    Point center((rear.x + front.x) / 2, (rear.y + front.y) / 2);
    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);

    float wheel_length = L / 4.0f;
    float wheel_width = d / 4.0f;
    float hull_length = L;
    float hull_width = 2 * d;

    // Hull corners in local frame (with wheel padding)
    std::vector<Point> local_hull = {
        {(-hull_length / 2) - wheel_length, (-hull_width / 2) - wheel_width},
        {(hull_length / 2) + wheel_length, (-hull_width / 2) - wheel_width},
        {(hull_length / 2) + wheel_length, (hull_width / 2) + wheel_width},
        {(-hull_length / 2) - wheel_length, (hull_width / 2) + wheel_width}
    };

    // Convert to image space polygon
    std::vector<cv::Point> hull_px;
    for (const Point &local : local_hull)
    {
        float wx = center.x + local.x * cos_theta - local.y * sin_theta;
        float wy = center.y + local.x * sin_theta + local.y * cos_theta;
        IndexPair ip = world->worldToIndices(Point(wx, wy));
        hull_px.push_back(cv::Point(ip.c, ip.r));
    }

    // Compute bounding box in image space
    cv::Rect bbox = cv::boundingRect(hull_px);

    for (int r = bbox.y; r < bbox.y + bbox.height; ++r)
    {
        for (int c = bbox.x; c < bbox.x + bbox.width; ++c)
        {
            cv::Point test_pt(c, r);
            // Check if inside hull polygon
            if (cv::pointPolygonTest(hull_px, test_pt, false) >= 0)
            {
                IndexPair ip(r, c);
                if (!world->isInside(ip))
                    return true;
                if (world->at(ip) < 127)
                    return true;
            }
        }
    }

    return false;
}


void CarRobot::draw()
{

    Point rear = pose.translation(); // Rear axle center
    float theta = pose.rotation();   // Heading angle (radians)

    float scale = world->inv_res;

    // Direction vectors
    Point dir(std::cos(theta), std::sin(theta));    // heading (forward)mo ve
    Point ortho(-std::sin(theta), std::cos(theta)); // lateral (left/right)

    Point front(rear.x + dir.x * L, rear.y + dir.y * L);

    // Draw axle line from rear to front
    IndexPair i_rear = world->worldToIndices(rear);
    IndexPair i_front = world->worldToIndices(front);
    cv::line(world->_display_image, cv::Point(i_rear.c, i_rear.r), cv::Point(i_front.c, i_front.r), cv::Scalar(128), 1);

    // Draw contact lines (across wheels) at both axles
    for (auto axle_center : {rear, front})
    {
        Point c1(axle_center.x - ortho.x * d, axle_center.y - ortho.y * d);
        Point c2(axle_center.x + ortho.x * d, axle_center.y + ortho.y * d);

        IndexPair i_c1 = world->worldToIndices(c1);
        IndexPair i_c2 = world->worldToIndices(c2);
        cv::line(world->_display_image, cv::Point(i_c1.c, i_c1.r), cv::Point(i_c2.c, i_c2.r), cv::Scalar(128), 1);
    }

    // Draw 4 wheels
    float wheel_length = (int)L / 4; // meters (along wheel direction)
    float wheel_width = (int)d / 4;  // meters (across axle)

    for (auto axle_center : {rear, front})
    {
        bool is_front = (axle_center.x == front.x && axle_center.y == front.y);
        float wheel_theta = theta + (is_front ? phi : 0.0f);

        float cos_w = std::cos(wheel_theta);
        float sin_w = std::sin(wheel_theta);

        for (float side : {-1.f, 1.f})
        {
            // Wheel center in world coordinates
            Point center(
                axle_center.x + ortho.x * side * d,
                axle_center.y + ortho.y * side * d);

            // Define wheel corners in local frame: (dx, dy) from center
            std::vector<cv::Point> corners_px;
            std::vector<Point> local_corners = {
                {-wheel_length / 2, -wheel_width / 2},
                {wheel_length / 2, -wheel_width / 2},
                {wheel_length / 2, wheel_width / 2},
                {-wheel_length / 2, wheel_width / 2}};

            for (const Point &local : local_corners)
            {
                // Rotate and translate to world coordinates
                float wx = center.x + local.x * cos_w - local.y * sin_w;
                float wy = center.y + local.x * sin_w + local.y * cos_w;

                IndexPair ip = world->worldToIndices(Point(wx, wy));
                corners_px.push_back(cv::Point(ip.c, ip.r));
            }

            // Draw filled rotated rectangle (wheel)
            cv::fillConvexPoly(world->_display_image, corners_px, cv::Scalar(0)); // black
        }
    }

    // Draw red dot at rear axle center
    IndexPair i_center = world->worldToIndices(rear);
    cv::circle(
        world->_display_image,
        cv::Point(i_center.c, i_center.r),
        10,                    // radius in pixels
        cv::Scalar(0, 0, 255), // red color (BGR format)
        -1                     // filled
    );

    // draw point at the translation of the pose
    Point translation = pose.translation();
    IndexPair i_translation = world->worldToIndices(translation);
    cv::circle(
        world->_display_image,
        cv::Point(i_translation.c, i_translation.r),
        5,                     // radius in pixels
        cv::Scalar(255, 0, 0), // blue color (BGR
        -1                     // filled
    );

    // Draw the Hull
    float hull_length = L;    // full car length
    float hull_width = 2 * d; // full car width

    // Center of hull is halfway between front and rear
    Point center((rear.x + front.x) / 2, (rear.y + front.y) / 2);

    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);

    std::vector<cv::Point> hull_pts;
    std::vector<Point> local_hull = {
        {(-hull_length / 2)- (wheel_length), (-hull_width / 2) - (wheel_width)},
        {(hull_length / 2)+ (wheel_length), -hull_width / 2 - (wheel_width)},
        {(hull_length / 2)+ (wheel_length), hull_width / 2 + (wheel_width)},
        {(-hull_length / 2)- (wheel_length), hull_width / 2 + (wheel_width)},
    };

    for (const Point &local : local_hull)
    {
        float wx = center.x + local.x * cos_theta - local.y * sin_theta;
        float wy = center.y + local.x * sin_theta + local.y * cos_theta;
        IndexPair ip = world->worldToIndices(Point(wx, wy));
        hull_pts.push_back(cv::Point(ip.c, ip.r));
    }

    // Green line linking the hull points
    cv::line(world->_display_image, hull_pts[0], hull_pts[1], cv::Scalar(0, 255, 0), 2);
    cv::line(world->_display_image, hull_pts[1], hull_pts[2], cv::Scalar(0, 255, 0), 2);
    cv::line(world->_display_image, hull_pts[2], hull_pts[3], cv::Scalar(0, 255, 0), 2);
    cv::line(world->_display_image, hull_pts[3], hull_pts[0], cv::Scalar(0, 255, 0), 2);    
    // cv::fillConvexPoly(world->_display_image, hull_pts, cv::Scalar(0, 255, 0));
}