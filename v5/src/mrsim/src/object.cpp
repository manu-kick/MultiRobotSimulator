#include <iostream>
#include <opencv2/imgproc.hpp>

#include "mrsim/object.h"
#include "mrsim/simple_geometry.h"

using namespace std;

void BoxObject::timeTick(float dt)
{
}

void BoxObject::drawPrehensile(Prehensile_Point &pp, const std::vector<cv::Point> &corners_px)
{
    if (corners_px.size() < 2)
        return;

    float value = pp.value;
    if (value < 0.0f || value > 100.0f)
    {
        std::cerr << "Invalid prehensile point value: " << value << std::endl;
        return;
    }

    // Promote to float for precision
    std::vector<cv::Point2f> pts;
    pts.reserve(corners_px.size());
    for (const auto &p : corners_px)
        pts.emplace_back((float)p.x, (float)p.y);
    const int N = (int)pts.size();

    auto edgeLen = [](const cv::Point2f &a, const cv::Point2f &b)
    {
        return std::hypot(b.x - a.x, b.y - a.y);
    };

    // Total perimeter
    float perimeter = 0.f;
    for (int i = 0; i < N; ++i)
        perimeter += edgeLen(pts[i], pts[(i + 1) % N]);
    if (perimeter <= 0.f)
        return;

    // Target arclength along perimeter
    float target = (value / 100.f) * perimeter;
    if (target >= perimeter)
        target = 0.f;

    // Centroid (for "outward" normal disambiguation)
    cv::Point2f C(0.f, 0.f);
    for (const auto &p : pts)
    {
        C.x += p.x;
        C.y += p.y;
    }
    C.x /= N;
    C.y /= N;

    // March along edges to find the segment containing 'target'
    for (int i = 0; i < N; ++i)
    {
        const cv::Point2f A = pts[i];
        const cv::Point2f B = pts[(i + 1) % N];
        const float L = edgeLen(A, B);

        if (target <= L || i == N - 1)
        {
            // Interpolate along this edge to get point P on perimeter
            float t = (L > 0.f) ? (target / L) : 0.f;
            cv::Point2f P(A.x + t * (B.x - A.x),
                          A.y + t * (B.y - A.y));

            // Unit tangent along edge
            cv::Point2f T = (L > 0.f) ? cv::Point2f((B.x - A.x) / L, (B.y - A.y) / L)
                                      : cv::Point2f(1.f, 0.f);

            // Two candidate normals (left/right)
            cv::Point2f n_left(-T.y, T.x);
            cv::Point2f n_right(T.y, -T.x);

            // Choose outward: the one pointing away from centroid
            cv::Point2f vPC(P.x - C.x, P.y - C.y);
            cv::Point2f Nrm = (n_left.x * vPC.x + n_left.y * vPC.y) >= 0.f ? n_left : n_right;

            // Length of the normal line must equal the selected edge length
            cv::Point2f Q(P.x + Nrm.x * L, P.y + Nrm.y * L);

            // Save Prehensile coords
            pp.grasp_px = Point(cvRound(Q.x), cvRound(Q.y));
            // cout<<"Saving grasping point x="<<pp.grasp_px.x<<" y="<<pp.grasp_px.y<<endl;

            cv::Scalar secondaryColorScalar = secondaryColor.toScalar();
            // Normal and circle on prehensile point
            cv::line(world->_display_image,
                     cv::Point(cvRound(P.x), cvRound(P.y)),
                     cv::Point(cvRound(Q.x), cvRound(Q.y)),
                     secondaryColorScalar, 2);

            cv::circle(world->_display_image,
                       cv::Point(cvRound(Q.x), cvRound(Q.y)),
                       5, secondaryColorScalar, -1);

            // 🔴 Draw Prehensile area
            int half = (int)(pp.grasp_area_size / 2.0f);
            cv::Point top_left(cvRound(Q.x) - half, cvRound(Q.y) - half);
            cv::Point bottom_right(cvRound(Q.x) + half, cvRound(Q.y) + half);
            cv::rectangle(world->_display_image, top_left, bottom_right, cv::Scalar(0, 0, 255), 1);

            return;
        }
        target -= L;
    }
}

void BoxObject::draw()
{
    Pose world_pose = pose;
    Point translation = world_pose.translation();
    IndexPair i_translation = world->worldToIndices(translation);

    // cout<<"Drawing BoxObject at (" << translation.x << ", " << translation.y << ")" << endl;

    // define the corners of the box in local coordinates
    std::vector<cv::Point> corners_px;
    corners_px.push_back(cv::Point(i_translation.c - width / 2, i_translation.r - height / 2));
    corners_px.push_back(cv::Point(i_translation.c + width / 2, i_translation.r - height / 2));
    corners_px.push_back(cv::Point(i_translation.c + width / 2, i_translation.r + height / 2));
    corners_px.push_back(cv::Point(i_translation.c - width / 2, i_translation.r + height / 2));

    // draw the box as a filled polygon

    cv::fillConvexPoly(world->_display_image, corners_px, primaryColor.toScalar());

    // draw the edges of the box
    cv::Scalar secondaryColorScalar = secondaryColor.toScalar();
    cv::line(world->_display_image, corners_px[0], corners_px[1], secondaryColorScalar, 2);
    cv::line(world->_display_image, corners_px[1], corners_px[2], secondaryColorScalar, 2);
    cv::line(world->_display_image, corners_px[2], corners_px[3], secondaryColorScalar, 2);
    cv::line(world->_display_image, corners_px[3], corners_px[0], secondaryColorScalar, 2);

    // draw the prehensile points
    for (auto &pp : prehensile_points)
    {
        // For simplicity, we draw prehensile points as small circles
        // std::cout << "Prehensile Point: " << pp.id << " with value: " << pp.value << std::endl;
        drawPrehensile(pp, corners_px);
    }

    // GOAL AREA
    // Draw lines between consecutive goal points
    for (size_t i = 1; i < goal.size(); ++i)
    {
        IndexPair idx1 = world->worldToIndices(goal[i - 1]);
        IndexPair idx2 = world->worldToIndices(goal[i]);
        cv::line(world->_display_image,
                 cv::Point(idx1.r, idx1.c),
                 cv::Point(idx2.r, idx2.c),
                 secondaryColorScalar, 2);
    }
    // Draw line from last goal point to first
    if (goal.size() > 1)
    {
        IndexPair idx1 = world->worldToIndices(goal.back());
        IndexPair idx2 = world->worldToIndices(goal.front());
        cv::line(world->_display_image,
                 cv::Point(idx1.r, idx1.c),
                 cv::Point(idx2.r, idx2.c),
                 secondaryColorScalar, 2);
    }

    // up the first vertex write as text "goal area"
    std::string goal_text = "goal_area";
    if (!goal.empty())
    {
        IndexPair idx0 = world->worldToIndices(goal.front());
        cv::Point text_origin(idx0.r, idx0.c);

        // shift text a bit upward/left for readability
        text_origin.y -= 10;
        text_origin.x -= 20;

        cv::putText(world->_display_image,
                    goal_text,
                    text_origin,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,                 // font scale
                    secondaryColorScalar, // text color (black)
                    1,                   // thickness
                    cv::LINE_AA);
    }
}

bool Object::isInsideGraspArea(const cv::Point &effector_px, std::string &grasp_id)
{
    for (const auto &pp : prehensile_points)
    {
        if (pp.grasp_px.x < 0 || pp.grasp_px.y < 0)
            continue; // non ancora calcolato

        int half = (int)(pp.grasp_area_size / 2.0f);
        cv::Rect grasp_rect(pp.grasp_px.x - half,
                            pp.grasp_px.y - half,
                            pp.grasp_area_size,
                            pp.grasp_area_size);
        if (grasp_rect.contains(effector_px))
        {
            grasp_id = pp.id;
            return true;
        }
    }
    return false;
}

bool Object::isInsideGoalArea(const Point &p)
{
    if (goal.size() < 3)
        return false; // not a valid polygon

    // Convert goal points to cv::Point
    std::vector<cv::Point> polygon;
    for (const auto &pt : goal)
    {
        IndexPair idx = world->worldToIndices(pt);
        polygon.emplace_back(idx.c, idx.r); // Note: cv::Point(x, y) corresponds to (col, row)
        // cout << "Goal Point: " << pt.x << ", " << pt.y << " -> Pixel: " << idx.c << ", " << idx.r << std::endl;
    }

    // Convert point p to cv::Point
    IndexPair p_idx = world->worldToIndices(p);
    cv::Point test_point(p_idx.r, p_idx.c);
    // cout << "Testing Point: " << p.x << ", " << p.y << " -> Pixel: " << p_idx.c << ", " << p_idx.r << std::endl;

    // Use cv::pointPolygonTest to check if the point is inside the polygon
    double result = cv::pointPolygonTest(polygon, test_point, false);
    if (result >= 0)
    {
        // std::cout << "Point " << p.x << ", " << p.y << " is inside the goal area." << std::endl;
        return true;
    }
    else
    {
        // std::cout << "Point " << p.x << ", " << p.y << " is outside the goal area." << std::endl;
        return false;
    }
}

bool Object::collides()
{
    // given the object subtype instance (BoxObject, others...)
    // call the appropriate collides() method
    if (BoxObject *box = dynamic_cast<BoxObject *>(this))
    {
        return box->collides();
    }
    else
    {
        throw std::runtime_error("collides() not implemented for this Object subtype");
    }
    return false;
}

bool BoxObject::isPointCollidingWithBoxObject(const Point &p)
{
    if (!world)
        return false;

    // (debug) point & object pose
    // std::cout << "P.x = " << p.x << " P.y = " << p.y
    //           << "  pose obj = " << pose.x << ", " << pose.y << std::endl;

    // Box size in world units (width/height are in pixels)
    float width_m  = width  * world->resolution;
    float height_m = height * world->resolution;

    // Axis-aligned corners in WORLD coordinates (centered at pose.x, pose.y)
    std::vector<Point> world_corners = {
        {pose.x - width_m * 0.5f, pose.y - height_m * 0.5f},
        {pose.x + width_m * 0.5f, pose.y - height_m * 0.5f},
        {pose.x + width_m * 0.5f, pose.y + height_m * 0.5f},
        {pose.x - width_m * 0.5f, pose.y + height_m * 0.5f}
    };

    // Convert corners to PIXEL coordinates for OpenCV polygon test
    std::vector<cv::Point> hull_px;
    hull_px.reserve(world_corners.size());
    for (const Point &corner : world_corners)
    {
        IndexPair ip = world->worldToIndices(corner);
        hull_px.emplace_back(ip.c, ip.r);  // (x=col, y=row)
    }

    // Convert test point to PIXEL coordinates
    IndexPair pip = world->worldToIndices(p);
    if (!world->isInside(pip))
        return false; // outside map → not considered colliding with on-map box

    cv::Point test_pt(pip.c, pip.r);

    // Inside (>=0 includes edges) ⇒ colliding with this box object
    return cv::pointPolygonTest(hull_px, test_pt, /*measureDist=*/false) >= 0;
}


bool BoxObject::collides()
{
    if (!world)
        return false;

    // 1) Build box corners in world coords (centered at pose.x, pose.y)
    // width is in pixels, convert to meters
    float width_m = width * world->resolution;
    float height_m = height * world->resolution;
    std::vector<Point> world_corners = {
        {pose.x - width_m / 2, pose.y - height_m / 2},
        {pose.x + width_m / 2, pose.y - height_m / 2},
        {pose.x + width_m / 2, pose.y + height_m / 2},
        {pose.x - width_m / 2, pose.y + height_m / 2}};

    // 2) Convert corners to pixel coords for OpenCV polygon
    std::vector<cv::Point> hull_px;
    hull_px.reserve(world_corners.size());
    for (const Point &corner : world_corners)
    {
        IndexPair ip = world->worldToIndices(corner);
        // print the corners in pixel coordinates
        hull_px.emplace_back(ip.c, ip.r);
    }

    // 3) Compute bounding box in pixel space
    cv::Rect bbox = cv::boundingRect(hull_px);
    

    // 4) Iterate only inside bbox and test polygon membership
    for (int r = bbox.y; r < bbox.y + bbox.height; ++r)
    {
        for (int c = bbox.x; c < bbox.x + bbox.width; ++c)
        {
            cv::Point test_pt(c, r);

            // Is this pixel inside the box polygon?
            if (cv::pointPolygonTest(hull_px, test_pt, false) >= 0)
            {
                IndexPair ip(r, c);
                if (!world->isInside(ip))
                {
                    cout << "c = " << c << " r = " << r << endl;
                    cout << "\t\t !!!!Cannot move object, collision detected, world boundaries violated!!!!" << endl;
                    return true;
                }
                if (world->at(ip) < 127)
                {
                    cout << "\t\t !!!!Cannot move object, collision detected, world obstacles hit!!!!" << endl;
                    return true;
                }
            }
        }
    }

    return false;
}