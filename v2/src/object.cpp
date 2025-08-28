#include "object.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
using namespace std;

// Example of a derived class for a box object
void BoxObject::timeTick(float dt)
{
    // Implement box-specific behavior here
    // For now, we do nothing
}

void BoxObject::drawPrehensile(const Prehensile_Point& pp,
                               const std::vector<cv::Point>& corners_px)
{
    if (corners_px.size() < 2) return;

    float value = pp.value;
    if (value < 0.0f || value > 100.0f) {
        std::cerr << "Invalid prehensile point value: " << value << std::endl;
        return;
    }

    // Promote to float for precision
    std::vector<cv::Point2f> pts;
    pts.reserve(corners_px.size());
    for (const auto& p : corners_px) pts.emplace_back((float)p.x, (float)p.y);
    const int N = (int)pts.size();

    auto edgeLen = [](const cv::Point2f& a, const cv::Point2f& b) {
        return std::hypot(b.x - a.x, b.y - a.y);
    };

    // Total perimeter
    float perimeter = 0.f;
    for (int i = 0; i < N; ++i) perimeter += edgeLen(pts[i], pts[(i + 1) % N]);
    if (perimeter <= 0.f) return;

    // Target arclength along perimeter
    float target = (value / 100.f) * perimeter;
    if (target >= perimeter) target = 0.f;

    // Centroid (for "outward" normal disambiguation)
    cv::Point2f C(0.f, 0.f);
    for (const auto& p : pts) { C.x += p.x; C.y += p.y; }
    C.x /= N; C.y /= N;

    // March along edges to find the segment containing 'target'
    for (int i = 0; i < N; ++i) {
        const cv::Point2f A = pts[i];
        const cv::Point2f B = pts[(i + 1) % N];
        const float L = edgeLen(A, B);

        if (target <= L || i == N - 1) {
            // Interpolate along this edge to get point P on perimeter
            float t = (L > 0.f) ? (target / L) : 0.f;
            cv::Point2f P(A.x + t * (B.x - A.x),
                          A.y + t * (B.y - A.y));

            // Unit tangent along edge
            cv::Point2f T = (L > 0.f) ? cv::Point2f((B.x - A.x)/L, (B.y - A.y)/L)
                                      : cv::Point2f(1.f, 0.f);

            // Two candidate normals (left/right)
            cv::Point2f n_left(-T.y,  T.x);
            cv::Point2f n_right( T.y, -T.x);

            // Choose outward: the one pointing away from centroid
            cv::Point2f vPC(P.x - C.x, P.y - C.y);
            cv::Point2f Nrm = (n_left.x * vPC.x + n_left.y * vPC.y) >= 0.f ? n_left : n_right;

            // Length of the normal line must equal the selected edge length
            cv::Point2f Q(P.x + Nrm.x * L, P.y + Nrm.y * L);

            // Draw normal line and end circle
            cv::line(world->_display_image,
                     cv::Point(cvRound(P.x), cvRound(P.y)),
                     cv::Point(cvRound(Q.x), cvRound(Q.y)),
                     cv::Scalar(103,50,5), 2);

            cv::circle(world->_display_image,
                       cv::Point(cvRound(Q.x), cvRound(Q.y)),
                       5, cv::Scalar(103,50,5), -1);

            // (Optional) debug:
            // std::cout << "PP " << pp.id << " @" << value << "%, edge " << i
            //           << " | L=" << L << " | P=(" << P.x << "," << P.y
            //           << ") Q=(" << Q.x << "," << Q.y << ")\n";
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
    
    cv::fillConvexPoly(world->_display_image, corners_px, cv::Scalar(208,102,13)); 

    //draw the edges of the box
    cv::line(world->_display_image, corners_px[0], corners_px[1], cv::Scalar(103,50,5), 2);
    cv::line(world->_display_image, corners_px[1], corners_px[2], cv::Scalar(103,50,5), 2);
    cv::line(world->_display_image, corners_px[2], corners_px[3], cv::Scalar(103,50,5), 2);
    cv::line(world->_display_image, corners_px[3], corners_px[0], cv::Scalar(103,50,5), 2);

    // draw the prehensile points
    for (const auto& pp : prehensile_points) {
        // For simplicity, we draw prehensile points as small circles
        // std::cout << "Prehensile Point: " << pp.id << " with value: " << pp.value << std::endl;
        drawPrehensile(pp, corners_px);
    }

    // GOAL AREA
    // Draw lines between consecutive goal points
    for (size_t i = 1; i < goal.size(); ++i) {
        IndexPair idx1 = world->worldToIndices(goal[i - 1]);
        IndexPair idx2 = world->worldToIndices(goal[i]);
        cv::line(world->_display_image,
                 cv::Point(idx1.r, idx1.c),
                 cv::Point(idx2.r, idx2.c),
                 cv::Scalar(0, 255, 0), 2);
    }
    // Draw line from last goal point to first
    if (goal.size() > 1) {
        IndexPair idx1 = world->worldToIndices(goal.back());
        IndexPair idx2 = world->worldToIndices(goal.front());
        cv::line(world->_display_image,
                 cv::Point(idx1.r, idx1.c),
                 cv::Point(idx2.r, idx2.c),
                 cv::Scalar(0, 255, 0), 2);
    }

    // up the first vertex write as text "goal area"
    std::string goal_text = "goal_area";
    if (!goal.empty()) {
        IndexPair idx0 = world->worldToIndices(goal.front());
        cv::Point text_origin(idx0.r, idx0.c);

        // shift text a bit upward/left for readability
        text_origin.y -= 10;
        text_origin.x -= 20;

        cv::putText(world->_display_image,
                    goal_text,
                    text_origin,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,                       // font scale
                    cv::Scalar(0, 0, 0),       // text color (black)
                    1,                         // thickness
                    cv::LINE_AA);
    }





}