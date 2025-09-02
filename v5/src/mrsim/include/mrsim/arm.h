#pragma once
#include <vector>
#include "world.h" // for WorldItem
#include "object.h"
#include "simple_geometry.h"

using namespace std;

// ----------------------- Link -----------------------
struct Link{
    float length; // Length of the link
    float angle;  // Angle of the link in radians
    int type;     // Type of the link (e.g., 0="link", 1="end_effector")
    Link(float length_=1.0f, float angle_=0.0f, int type_=0)
        : length(length_), angle(angle_), type(type_) {}
};

// ------------------- EndEffector --------------------
struct EndEffector {
    bool is_holding = false;
    bool gripper_closed = false;
    Object* held_object = nullptr;
    string saved_grasp_id; // store the grasp id used at docking time
    bool is_closing_gripper = false;

    Point     position;    // world coords (mappa/metri nel tuo sistema)
    IndexPair pixel_pos;   // pixel coords (per check nelle grasp areas)
    float     orientation = 0.f; // orientamento EE (rad), utile per orientare l’oggetto

    bool dock_request = false; // <-- richiesta di docking (EndEffector non vede il world)

    EndEffector() : position(0,0), pixel_pos(0,0) {}
    EndEffector(bool is_holding_, Object* held_object_ = nullptr)
        : is_holding(is_holding_), held_object(held_object_), position(0,0), pixel_pos(0,0) {}

    // Ora "hold" alza solo la richiesta: l'Arm la consumerà in timeTick()
    void hold() {
        if (!is_holding) dock_request = true;
    }

    void release() {
        // controlla se l'oggetto è nella goal area
        if (is_holding && held_object) {
            auto obj_center = held_object->pose.translation();
            if (held_object->isInsideGoalArea(obj_center)) {
                std::cout << "\t-Object released inside goal area!" << std::endl;
                // lock the obj
                held_object->locked = true;
                held_object->isHeld = false;
            } else {
                std::cout << "\t-Object released outside goal area!" << std::endl;
            }
        }
                

        is_holding = false;
        held_object = nullptr;
        saved_grasp_id.clear();
        gripper_closed = false;
        // (l’Arm si occupa di ripristinare lunghezza link finale)
    }
    bool isHolding()        const { return is_holding; }
    bool isClosingGripper() const { return is_closing_gripper; }

    WorldItem* getHeldObject() const { return held_object; }
};

// ------------------------ Arm -----------------------
struct Arm: public WorldItem {
    std::vector<Link> links;
    EndEffector end_effector; // end effector
    WorldItem* parent_robot;

    // --- Holding offset (prehensile point relative to object center, in object/local frame)
    Point hold_offset_local = Point(0,0);
    bool  has_hold_offset   = false;


    // Conserviamo le lunghezze originali per il release
    std::vector<float> original_lengths;

    Arm(WorldItem* parent_robot_ = nullptr,
        const std::vector<Link>& links_ = {},
        const EndEffector& end_effector_ = {})
        : WorldItem(parent_robot_),
          links(links_),
          end_effector(end_effector_),
          parent_robot(parent_robot_) {
        original_lengths.reserve(links.size());
        for (const auto& L : links) original_lengths.push_back(L.length);
    }

    void timeTick(float dt);
    bool collides();
    void draw();

    // ---- Nuove utilità per docking/release ----
    // Cerca un oggetto la cui grasp area contenga l'EE; se trovato, restituisce obj e grasp_id
    bool dock(Object*& out_obj, std::string& out_grasp_id);

    // Allinea e (se serve) allunga il last link per portare l’EE su target_world
    void alignLastLinkTo(const Point& target_world);

    // Ripristina ultimo link e rilascia oggetto
    void doRelease();

    // Compute current end-effector world pose from links (no drawing side-effects)
    void computeEndEffectorPose(Point& out_pos, float& out_theta) const;
};

