#include <iostream>
#include <fstream>
#include <json/json.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include <memory>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include "mrsim/sim_common.h"
#include "mrsim/ros_bridge.h"
#include "mrsim/world.h"
#include "mrsim/utils.h"
#include "mrsim/arm.h"
#include "mrsim/simple_geometry.h"
#include "mrsim/lidar.h"
#include "mrsim/freeflying.h"
#include "mrsim/car.h"

using namespace std;

void placeItems(World &world, const Json::Value &root, std::vector<RobotHandle> &outRobots)
{
    const Json::Value robots = root["robots"];
    cout << "******************* CREATING ROBOT **********************" << endl;
    for (Json::ArrayIndex i = 0; i < robots.size(); ++i)
    {
        const Json::Value &r = robots[i];
        std::string id = r.get("id", std::to_string(i + 1)).asString();
        std::string type = r.get("type", "freeflying").asString();

        // default pose; override if present in JSON
        Pose p(25, 20, M_PI / 2);
        if (r.isMember("initial_pose") && r["initial_pose"].isObject())
        {
            const auto &pj = r["initial_pose"];
            p.x = pj.get("x", p.x).asDouble();
            p.y = pj.get("y", p.y).asDouble();
            p.theta = pj.get("theta", p.theta).asDouble();
        }

        if (type == "freeflying")
        {
            auto *ff = new FreeFlying(&world, 1);
            ff->pose = p;
            world.addItem(ff);
            outRobots.push_back({ff, id, type, p});
            cout << "\t - FreeFlying " << id << " @ (" << p.x << "," << p.y << "," << p.theta << ")\n";

            // Load arm if present
            if (r.isMember("arm") && r["arm"].isArray())
            {
                std::vector<Link> links;
                EndEffector end_effector; // Optional end effector
                for (const auto &link : r["arm"])
                {
                    float length = link.get("length", 1.0f).asFloat();
                    float angle = link.get("initial_angle", 0.0f).asFloat();
                    int type = link.get("type", 0).asInt();
                    if (type == 0)
                    {
                        // Regular link
                        links.emplace_back(length, angle, type);
                    }
                    else
                    {
                        // End effector
                        end_effector = EndEffector();
                    }
                }

                auto *arm = new Arm(ff, links, end_effector);
                // arm->end_effector.hold(ff); // Optional: hold the FreeFlying itself
                // arm->pose = p;
                ff->arm = arm; // Explicit reference to the arm
                world.addItem(arm);
                std::cout << "\t - Arm with " << links.size() << " links added to FreeFlying " << id << "\n";
            }
        }
        else if (type == "car")
        {
            // throw std::runtime_error("ARM on CarRobot not implemented yet");
            auto *car = new CarRobot(&world);
            car->pose = p;
            world.addItem(car);
            outRobots.push_back({car, id, type, p});
            std::cout << "\t - CarRobot " << id << " @ (" << p.x << "," << p.y << "," << p.theta << ")\n";
        }
        else
        {
            std::cerr << "\t - Unknown robot type: " << type << " (skipped)\n";
        }
    }
    cout << endl
         << endl;
}

void placeObjects(World &world, const Json::Value &root, std::vector<Object *> &outObjects)
{

    const Json::Value objects = root["objects"];
    cout << "******************* CREATING OBJECTS **********************" << endl;
    for (Json::ArrayIndex i = 0; i < objects.size(); ++i)
    {
        const Json::Value &o = objects[i];
        std::string id = o.get("id", std::to_string(i + 1)).asString();
        std::string type = o.get("type", "box").asString();

        // default pose; override if present in JSON
        Pose p(25, 20, M_PI / 2);
        float width = 10.0f, height = 10.0f;
        if (o.isMember("size") && o["size"].isObject())
        {
            const auto &size = o["size"];
            width = size.get("width", width).asFloat();
            height = size.get("height", height).asFloat();
        }
        if (o.isMember("initial_pose") && o["initial_pose"].isObject())
        {
            const auto &pj = o["initial_pose"];
            p.x = pj.get("x", p.x).asDouble();
            p.y = pj.get("y", p.y).asDouble();
            p.theta = pj.get("theta", p.theta).asDouble();
        }

        std::vector<Prehensile_Point> list_prehensile;
        if (o.isMember("prehensile_points") && o["prehensile_points"].isArray())
        {
            const auto &prehensile_points = o["prehensile_points"];
            for (const auto &pp : prehensile_points)
            {
                Prehensile_Point prehensile_point(pp.get("id", "").asString(), pp.get("value", 0.0f).asFloat());
                list_prehensile.push_back(prehensile_point);
            }
        }

        std::vector<Point> goal_vertices;
        if (o.isMember("goal") && o["goal"].isArray())
        {
            const auto &goal = o["goal"];
            for (const auto &g : goal)
            {
                if (g.isObject())
                {
                    float x = g.get("x", 0).asFloat();
                    float y = g.get("y", 0).asFloat();

                    Point p(x, y);

                    goal_vertices.push_back(p);
                }
            }
        }

        if (type == "box")
        {
            auto *box = new BoxObject(&world, width, height, p, list_prehensile, goal_vertices);
            cout << "\t - BoxObject " << id << " @ (" << p.x << "," << p.y << "," << p.theta << ") size (" << width << "x" << height << ")\n";
            world.addItem(box);
            outObjects.push_back(box); // <-- add this line
        }
    }
    cout << endl
         << endl;
}

// Function responsible of controlling the arm of the selected robot if available
void controlArm(RobotHandle &robotHandle, Key key)
{
    if (key == Key::BackToRobot)
    { // handle first
        robotHandle.arm_index = -1;
        std::cout << "\t-Returning to robot control.\n";
        return;
    }

    Arm *arm = nullptr;

    if (auto *ff = dynamic_cast<FreeFlying *>(robotHandle.ptr))
    {
        if (ff->hasArm())
            arm = ff->arm;
    }
    else if (auto *car = dynamic_cast<CarRobot *>(robotHandle.ptr))
    {
        if (car->hasArm())
            arm = car->arm;
    }
    if (!arm)
        return;

    int num_links = static_cast<int>(arm->links.size());

    switch (key)
    {
    case Key::Left:
    { // increase angle
        if (robotHandle.arm_index != -1)
        {
            const int i = robotHandle.arm_index;
            const float old = arm->links[i].angle;
            arm->links[i].angle = old + 0.05f;
            if (arm->collides())
            {
                arm->links[i].angle = old;
                std::cout << "\t-Blocked (EE collision), rotate the other way.\n";
            }
        }
        break;
    }
    case Key::Right:
    { // decrease angle
        if (robotHandle.arm_index != -1)
        {
            const int i = robotHandle.arm_index;
            const float old = arm->links[i].angle;
            arm->links[i].angle = old - 0.05f;
            if (arm->collides())
            {
                arm->links[i].angle = old;
                std::cout << "\t-Blocked (EE collision), rotate the other way.\n";
            }
        }
        break;
    }

    case Key::BackToRobot:
    { // 'b'
        robotHandle.arm_index = -1;
        std::cout << "\t-Returning to robot control.\n";
        break;
    }

    case Key::ToggleHold:
    { // 'e'
        if (robotHandle.arm_index != -1)
        {
            if (arm->end_effector.isHolding())
            {
                arm->doRelease();
                std::cout << "\t-Release arm index: " << robotHandle.arm_index << std::endl;
            }
            else
            {
                arm->end_effector.hold(); // request docking
                std::cout << "\t-Docking request arm index: " << robotHandle.arm_index << std::endl;
            }
        }
        break;
    }

    case Key::Tab:
    { // select next link
        if (num_links > 0)
        {
            robotHandle.arm_index = (robotHandle.arm_index + 1) % num_links;
            std::cout << "\t-Selected arm index: " << robotHandle.arm_index << std::endl;
        }
        break;
    }

    case Key::Esc:
        std::cout << "Exiting simulator...." << std::endl;
        rclcpp::shutdown();

    default:
        break;
    }
}

int main(int argc, char **argv)
{

    const string rankingPath = "/home/ubuntu/Desktop/MultiRobotSimulator/v5/src/mrsim/rankings/ranking.json";
    PlayerInfo player = selectOrCreatePlayer(rankingPath);
    if (player.name.empty()) {
        std::cerr << "Player selection failed (ranking file not writable?). Exiting.\n";
        return 1;
    }

    rclcpp::init(argc, argv);
    
    cout << "Loading your favorite level " <<player.fav_level << endl;
    std::string jsonFilePath = "/home/ubuntu/Desktop/MultiRobotSimulator/v5/src/mrsim/configs/"+ std::to_string(player.fav_level) + ".json";

    Json::Value root;
    Json::Reader reader;

    ifstream jsonFile(jsonFilePath, ifstream::binary);

    if (!jsonFile.good())
    {
        cerr << "Error opening JSON file: " << jsonFilePath << endl;
        return 1;
    }

    if (!reader.parse(jsonFile, root))
    {
        cerr << "Error parsing JSON file: " << jsonFilePath << endl;
        return 1;
    }

    // Access to config
    string mapFile = root["map_file"].asString();
    const double res = root.get("world_resolution", 0.05).asDouble();
    const float delay = root.get("delay", 0.1).asDouble();

    cout << "Map file name: " << mapFile << endl;

    // --- Build world ---
    World w(res);
    w.loadFromImage(mapFile);

    // Collect robots here
    std::vector<RobotHandle> robots;
    std::vector<Object *> objects;

    placeItems(w, root, robots);
    placeObjects(w, root, objects);
    using Clock = std::chrono::steady_clock;

    bool timing_started = false;
    bool timing_done = false;
    Clock::time_point t0;
    int eligible_count = 0; // will store the time when all reached their goals

    // Count how many objects actually have a goal polygon (>=3 vertices)
    for (auto *o : objects)
    {
        if (o && o->goal.size() >= 3)
            ++eligible_count;
    }

    // Helper that checks if *all* eligible objects are inside their goal (by center)
    auto all_objects_in_goal = [&]() -> bool
    {
        if (eligible_count == 0)
            return false; // nothing to time
        for (auto *o : objects)
        {
            if (!o || o->goal.size() < 3)
                continue;
            if (!o->isInsideGoalArea(o->pose.translation()))
                return false;
            if (!o->locked)
                return false;
        }
        return true;
    };

    // If we start with everything already inside, report immediately (0 s)
    if (all_objects_in_goal())
    {
        cout << "\n>>> All objects are already in their goal areas (0 s).\n\n";
        timing_done = true;
    }

    // Create a ros Bridge
    auto ros_bridge = std::make_shared<RosBridge>(w, robots);

    // Display instructions
    int selected = 0; // 0-based index
    cout << "******************* INSTRUCTIONS **********************" << endl;
    cout << "Press ESC to exit, spacebar to stop, 'r' to reset pose." << endl;
    cout << "Press number 1.." << robots.size() << " to select robot (0 = #10).\n";
    cout << "Selected robot: " << (selected + 1) << " [" << robots[selected].id << "]\n";

    int k;
    bool game_over = false;
    while (!game_over)
    {
        ros_bridge->spinOnce();
        w.timeTick(delay);
        w.show();

        // --- WALL-CLOCK timing independent of JSON 'delay' ---
        auto now = Clock::now();
        bool all_in = all_objects_in_goal();

        // Start when there's actually work to do
        if (!timing_started && !timing_done && eligible_count > 0 && !all_in)
        {
            t0 = now;
            timing_started = true;
        }

        // Stop when everything is in
        if (timing_started && !timing_done && all_in)
        {
            double elapsed = std::chrono::duration<double>(now - t0).count();
            timing_done = true;
            std::cout << "\n>>> All objects reached their goal areas in "<< elapsed << " s (wall time).\n\n";
            // Save ranking data for the selected player
            if (!saveMatchResult(rankingPath, player, elapsed)) {
                std::cerr << "Warning: could not save match result.\n";
            }
            else{
                game_over = true;
                cout<<"Chage your favorite level in the ranking file"<<endl;
            }
        }

        // Wait for a key press
        k = cv::waitKeyEx(delay * 1000) & 255;
        Key key = normalizeKey(k);

        if (key >= Key::Digit1 && key <= Key::Digit9)
        {
            int idx = static_cast<int>(key) - static_cast<int>(Key::Digit0) - 1;
            if (idx < static_cast<int>(robots.size()))
            {
                selected = idx;
                std::cout << "Selected robot: " << (selected + 1) << " [" << robots[selected].id << "]\n";
            }
        }
        else if (key == Key::Digit0 && robots.size() >= 10)
        {
            selected = 9; // #10
            std::cout << "Selected robot: 10 [" << robots[selected].id << "]\n";
        }

        // --- Controls if NO arm control is active
        if (robots[selected].arm_index == -1)
        {
            switch (key)
            {
            case Key::Up:
            {
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->tv += 0.1f;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setVelocity(car->v + 0.1f);
                }
                break;
            }
            case Key::Down:
            {
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->tv -= 0.1f;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setVelocity(car->v - 0.1f);
                }
                break;
            }
            case Key::Left:
            {
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->rv += 0.1f;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setSteeringAngle(car->phi + 0.05f);
                }
                break;
            }
            case Key::Right:
            {
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->rv -= 0.1f;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setSteeringAngle(car->phi - 0.05f);
                }
                break;
            }
            case Key::Space:
            { // stop
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->tv = 0;
                    ff->rv = 0;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setVelocity(0);
                    car->setSteeringAngle(0);
                }
                break;
            }
            case Key::Reset:
            {
                robots[selected].ptr->pose = robots[selected].init;
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->tv = 0;
                    ff->rv = 0;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setVelocity(0);
                    car->setSteeringAngle(0);
                }
                std::cout << "Reset robot " << (selected + 1) << " [" << robots[selected].id << "]\n";
                break;
            }
            case Key::Esc:
                std::cout << "Exiting simulator...." << std::endl;
                rclcpp::shutdown();
                return 0;

            default:
                break;
            }
        }

        // Arm controls (now use normalized key)
        controlArm(robots[selected], key);
    }

    cout << "********************* Game Over! ************************";
    cout << "Change the favorite level in the ranking file to load another game" <<endl;

    return 0;
}