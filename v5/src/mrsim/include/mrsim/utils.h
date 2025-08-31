#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <json/json.h>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/imgproc.hpp>
#include "simple_geometry.h" // uses Point, Pose

using namespace std;
struct PlayerInfo {
    unsigned int id{0};
    std::string name;
    unsigned int fav_level{1};
};

PlayerInfo selectOrCreatePlayer(const std::string& rankingPath);
bool saveMatchResult(const std::string& rankingPath,
                     const PlayerInfo& player,
                     double elapsed_seconds);




// Colors for OpenCV drawing (BGR)
class Color {
public:
    int b, g, r;
    Color(int b_=0, int g_=0, int r_=0) : b(b_), g(g_), r(r_) {}

    inline static Color generateRandomColor() {
        return Color(rand() % 256, rand() % 256, rand() % 256);
    }

    inline cv::Scalar toScalar() const { return cv::Scalar(b, g, r); }

};

inline Color generateSecondaryColor(Color c) {
    int factor = 2; // darkening factor
    int b = std::max(0, c.b - factor * 30);
    int g = std::max(0, c.g - factor * 30);
    int r = std::max(0, c.r - factor * 30);
    return Color(b, g, r);
}

// --- Cross-platform key normalization for cv::waitKeyEx ---
enum class Key {
    None,
    Up, Down, Left, Right,
    Space, Esc, Reset, NextLink, ToggleHold, BackToRobot,
    Digit0, Digit1, Digit2, Digit3, Digit4, Digit5, Digit6, Digit7, Digit8, Digit9,
    Tab
};

inline Key normalizeKey(int k) {
    // --- Arrow keys FIRST (so 82 isn't mistaken for 'R') ---

    // Linux/X11 (often): left=81, up=82, right=83, down=84
    if (k == 82) return Key::Up;
    if (k == 84) return Key::Down;
    if (k == 81) return Key::Left;
    if (k == 83) return Key::Right;

    // macOS (some builds): up=0, down=1, left=2, right=3
    if (k == 0) return Key::Up;
    if (k == 1) return Key::Down;
    if (k == 2) return Key::Left;
    if (k == 3) return Key::Right;

    // Windows common OpenCV codes:
    // left=2424832, up=2490368, right=2555904, down=2621440
    if (k == 2490368) return Key::Up;
    if (k == 2621440) return Key::Down;
    if (k == 2424832) return Key::Left;
    if (k == 2555904) return Key::Right;

    // --- Printable AFTER arrows ---
    if (k == ' ') return Key::Space;
    if (k == 27)  return Key::Esc;     // ESC
    if (k == 'r' || k == 'R') return Key::Reset;
    if (k == 'e' || k == 'E') return Key::ToggleHold;
    if (k == 'b' || k == 'B') return Key::BackToRobot;
    if (k == '\t' || k == 9)  return Key::Tab;

    // Digits
    if (k >= '0' && k <= '9') {
        return static_cast<Key>(static_cast<int>(Key::Digit0) + (k - '0'));
    }

    return Key::None;
}


// a header both robots can see (e.g., simple_geometry.h or a new collision_utils.h)
inline float sqr(float x) { return x * x; }
inline float dist2(const Point &a, const Point &b)
{
    const float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}
inline Point carCenter(const Pose &pose, float L)
{
    const float th = pose.rotation();
    const Point rear = pose.translation();
    const Point dir(std::cos(th), std::sin(th));
    return Point(rear.x + 0.5f * L * dir.x, rear.y + 0.5f * L * dir.y);
}

inline float dot(const Point &a, const Point &b) { return a.x * b.x + a.y * b.y; }
inline Point sub(const Point &a, const Point &b) { return Point(a.x - b.x, a.y - b.y); }
inline Point add(const Point &a, const Point &b) { return Point(a.x + b.x, a.y + b.y); }
inline Point mul(const Point &a, float s) { return Point(a.x * s, a.y * s); }

inline float length(const Point &v) { return std::sqrt(dot(v, v)); }
inline Point normalize(const Point &v)
{
    float L = length(v);
    return (L > 0.f) ? Point(v.x / L, v.y / L) : Point(0.f, 0.f);
}

// Rectangle hull of the car (with wheel padding) in WORLD coordinates.
// Rear axle center is at `pose.translation()`, heading = pose.rotation().
inline std::vector<Point> carHullWorld(const Pose &pose, float L, float d, float wheel_len_pad, float wheel_wid_pad)
{
    const float th = pose.rotation();
    const float c = std::cos(th), s = std::sin(th);
    const Point rear = pose.translation();
    const Point dir(c, s);
    const Point front(rear.x + dir.x * L, rear.y + dir.y * L);
    const Point center(0.5f * (rear.x + front.x), 0.5f * (rear.y + front.y));

    const float hull_len = L;
    const float hull_wid = 2.f * d;
    const float ax = 0.5f * hull_len + wheel_len_pad; // x half-extent in local
    const float ay = 0.5f * hull_wid + wheel_wid_pad; // y half-extent in local

    // local corners (CCW)
    const Point local[4] = {
        Point(-ax, -ay), Point(+ax, -ay),
        Point(+ax, +ay), Point(-ax, +ay)};

    std::vector<Point> world;
    world.reserve(4);
    auto add_corner = [&](float lx, float ly)
    {
        float wx = center.x + lx * c - ly * s;
        float wy = center.y + lx * s + ly * c;
        world.emplace_back(wx, wy); // constructs in place, no default ctor needed
    };

    add_corner(-ax, -ay);
    add_corner(+ax, -ay);
    add_corner(+ax, +ay);
    add_corner(-ax, +ay);
    return world; // CCW quad
}

// SAT test for convex polygons in WORLD coordinates
inline bool convexPolygonsIntersectSAT(const std::vector<Point> &A,
                                       const std::vector<Point> &B)
{
    auto overlap_on_axis = [](const std::vector<Point> &P,
                              const Point &axis, float &minP, float &maxP)
    {
        minP = maxP = dot(P[0], axis);
        for (size_t i = 1; i < P.size(); ++i)
        {
            float p = dot(P[i], axis);
            if (p < minP)
                minP = p;
            if (p > maxP)
                maxP = p;
        }
    };

    auto check_axes = [&](const std::vector<Point> &P, const std::vector<Point> &Q) -> bool
    {
        for (size_t i = 0; i < P.size(); ++i)
        {
            const Point e = sub(P[(i + 1) % P.size()], P[i]);
            // normal axis (perp to edge)
            Point axis(-e.y, e.x);
            axis = normalize(axis);
            if (axis.x == 0.f && axis.y == 0.f)
                continue;

            float minP, maxP, minQ, maxQ;
            overlap_on_axis(P, axis, minP, maxP);
            overlap_on_axis(Q, axis, minQ, maxQ);
            if (maxP < minQ || maxQ < minP)
                return false; // separating axis
        }
        return true;
    };

    return check_axes(A, B) && check_axes(B, A);
}

// Point-in-convex-polygon (CCW) test
inline bool pointInConvexPolygon(const std::vector<Point> &poly, const Point &p)
{
    // all cross products must have same sign (CCW polygon => positive)
    for (size_t i = 0; i < poly.size(); ++i)
    {
        const Point a = poly[i];
        const Point b = poly[(i + 1) % poly.size()];
        const Point ab = sub(b, a);
        const Point ap = sub(p, a);
        const float cross = ab.x * ap.y - ab.y * ap.x;
        if (cross < 0.f)
            return false;
    }
    return true;
}

inline float distPointToSegment(const Point &p, const Point &a, const Point &b)
{
    const Point ab = sub(b, a);
    const float L2 = dot(ab, ab);
    if (L2 == 0.f)
        return std::sqrt(dist2(p, a));
    float t = dot(sub(p, a), ab) / L2;
    t = std::max(0.f, std::min(1.f, t));
    const Point proj = add(a, mul(ab, t));
    return std::sqrt(dist2(p, proj));
}

// Convex polygon (CCW) vs circle (world)
inline bool convexPolygonIntersectsCircle(const std::vector<Point> &poly,
                                          const Point &c, float r)
{
    if (pointInConvexPolygon(poly, c))
        return true;
    const float r2 = r * r;
    // edge distance
    for (size_t i = 0; i < poly.size(); ++i)
    {
        const Point &a = poly[i];
        const Point &b = poly[(i + 1) % poly.size()];
        float d = distPointToSegment(c, a, b);
        if (d <= r)
            return true;
    }
    // (optional) vertex check (covered by segment distance, but cheap)
    for (const Point &v : poly)
    {
        if (dist2(v, c) <= r2)
            return true;
    }
    return false;
}
