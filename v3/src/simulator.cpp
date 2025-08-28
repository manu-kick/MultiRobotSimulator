#include <iostream>
#include <fstream>
#include <json/json.h>
#include "opencv2/opencv.hpp"
#include "world.h"
#include "arm.h"
#include "simple_geometry.h"
#include "lidar.h"
#include "freeflying.h"
#include "car.h"
#include <vector>
#include <memory>

using namespace std;

// --- Handler for robot instances ---
struct RobotHandle
{
    WorldItem *ptr; // non-owning; World keeps it alive
    std::string id;
    std::string type;
    Pose init;

    // arm index (the one that is being controlled by the user)
    int arm_index{-1};

    RobotHandle() = default;
    RobotHandle(WorldItem *p, std::string id_, std::string type_, const Pose &init_, int arm_idx = -1)
        : ptr(p), id(std::move(id_)), type(std::move(type_)), init(init_), arm_index(arm_idx) {}
};

// Update signature to output the created robots
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
            auto *ff = new FreeFlying(&world, 2);
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

void placeObjects(World &world, const Json::Value &root, std::vector<Object> &obj)
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
            // obj.push_back(*box);
        }
    }
    cout << endl
         << endl;
}

// Function responsible of controlling the arm of the selected robot if available
void controlArm(RobotHandle &robotHandle, const int key)
{
    // cout << "Controlling arm for robot: [" << robotHandle.id << "] \t Pressed key: " << key << endl;
    // dynamic cast to correct type of robot
    Arm *arm = nullptr;

    // Try to get the arm from FreeFlying
    if (auto *ff = dynamic_cast<FreeFlying *>(robotHandle.ptr))
    {
        if (ff->hasArm())
            arm = ff->arm;
    }
    // Try to get the arm from CarRobot
    else if (auto *car = dynamic_cast<CarRobot *>(robotHandle.ptr))
    {
        if (car->hasArm())
            arm = car->arm;
    }

    if (!arm)
    {
        // No arm present, exit function
        return;
    }
    // get the number of links
    int num_links = arm->links.size();

    switch (key)
    {
        // case 2: // left arrow key to rotateclear left (increase  angle of the selected link)
        //     if (arm && robotHandle.arm_index != -1)
        //     {
        //         arm->links[robotHandle.arm_index].angle += 0.05;
        //     }
        //     break;

        // case 3: // right arrow key to rotate right (decrease angle of the selected link)
        //     if (arm && robotHandle.arm_index != -1)
        //     {
        //         arm->links[robotHandle.arm_index].angle -= 0.05;
        //     }
        // break;
        case 2: { // left arrow: try increase angle
            if (arm && robotHandle.arm_index != -1) {
                const int i = robotHandle.arm_index;
                const float old = arm->links[i].angle;
                arm->links[i].angle = old + 0.05f;         // propose
                if (arm->collides()) {                     // reject if colliding
                    arm->links[i].angle = old;
                    std::cout << "\t-Blocked (EE collision), rotate the other way.\n";
                }
            }
            break;
        }

        case 3: { // right arrow: try decrease angle
            if (arm && robotHandle.arm_index != -1) {
                const int i = robotHandle.arm_index;
                const float old = arm->links[i].angle;
                arm->links[i].angle = old - 0.05f;         // propose
                if (arm->collides()) {                     // reject if colliding
                    arm->links[i].angle = old;
                    std::cout << "\t-Blocked (EE collision), rotate the other way.\n";
                }
            }
            break;
        }


    case 98: // 'b' key to return to robot control
        // Reset arm index to -1 to indicate no arm control
        robotHandle.arm_index = -1;
        cout << "\t-Returning to robot control." << std::endl;
        break;

    case 101: // 'e' key to toggle end effector holding
        if (arm && robotHandle.arm_index != -1)
        {
            if (arm->end_effector.isHolding())
            {
                arm->doRelease(); // <-- invece di end_effector.release();
                std::cout << "\t-Release arm index: " << robotHandle.arm_index << std::endl;
            }
            else
            {
                arm->end_effector.hold(); // alza richiesta; Arm::timeTick farà il docking
                std::cout << "\t-Docking request arm index: " << robotHandle.arm_index << std::endl;
            }
        }
        break;

    case 9: // Tab key to select next link
        // Select next link
        robotHandle.arm_index = (robotHandle.arm_index + 1) % num_links;
        cout << "\t-Selected arm index: " << robotHandle.arm_index << std::endl;
        break;
    }
}

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        cerr << "Usage: <config_file.json>" << endl;
        return 1;
    }

    // We get the path of the JSON file from the arguments
    const string jsonFilePath = argv[1];

    Json::Value root;
    Json::Reader reader;

    // ifstream jsonFile(("/home/lattinone/RobotProgramming/Project/workspace/src/mrsim/test_data/" + jsonFilePath).c_str(), ifstream::binary);
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

    // We access the map file
    string mapFile = root["map_file"].asString();
    const double res = root.get("world_resolution", 0.05).asDouble();
    const float delay = root.get("delay", 0.1).asDouble();

    cout << "Map file name: " << mapFile << endl;

    // --- Build world ---
    World w(res);
    w.loadFromImage(mapFile);

    // Collect robots here
    std::vector<RobotHandle> robots;
    std::vector<Object> objects;

    placeItems(w, root, robots);
    placeObjects(w, root, objects);

    // Display instructions
    int selected = 0; // 0-based index
    cout << "******************* INSTRUCTIONS **********************" << endl;
    cout << "Press ESC to exit, spacebar to stop, 'r' to reset pose." << endl;
    cout << "Press number 1.." << robots.size() << " to select robot (0 = #10).\n";
    cout << "Selected robot: " << (selected + 1) << " [" << robots[selected].id << "]\n";

    int k;
    while (1)
    {
        w.timeTick(delay);
        w.show();

        // Wait for a key press
        k = cv::waitKeyEx(delay * 1000) & 255;

        // --- Selection via number keys
        if (k >= '1' && k <= '9')
        {
            int idx = k - '1';
            if (idx < (int)robots.size())
            {
                selected = idx;
                std::cout << "Selected robot: " << (selected + 1) << " [" << robots[selected].id << "]\n";
            }
        }
        else if (k == '0' && robots.size() >= 10)
        {
            selected = 9; // #10
            std::cout << "Selected robot: 10 [" << robots[selected].id << "]\n";
        }

        // --- Controls of the robot if the arm is not being controlled
        if (robots[selected].arm_index == -1) // No arm control
        {
            switch (k)
            {
#ifdef __APPLE__
            case 0:
            { // up
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->tv += 0.1;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setVelocity(car->v + 0.1);
                }
                break;
            }
            case 1:
            { // down
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->tv -= 0.1;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setVelocity(car->v - 0.1);
                }
                break;
            }
            case 2:
            { // left
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->rv += 0.1;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setSteeringAngle(car->phi + 0.05);
                }
                break;
            }
            case 3:
            { // right
                if (auto *ff = dynamic_cast<FreeFlying *>(robots[selected].ptr))
                {
                    ff->rv -= 0.1;
                }
                else if (auto *car = dynamic_cast<CarRobot *>(robots[selected].ptr))
                {
                    car->setSteeringAngle(car->phi - 0.05);
                }
                break;
            }
#endif

            case ' ':
            { // space -> stop
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

            case 'r':
            case 'R':
            { // reset pose of selected robot
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

            case 27: // ESC
                cout << "Exiting simulator...." << endl;
                return 0;

            default:
                break;
            }
        }

        // Arm controls
        controlArm(robots[selected], k);
    }

    return 0;
}