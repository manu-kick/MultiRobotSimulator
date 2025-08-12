#include <iostream>
#include <fstream>
#include <json/json.h>

using namespace std;

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
    string mapFileName = root["name"].asString();

    cout << "Map file name: " << mapFileName << endl;
    // Read the robots array
    const Json::Value robots = root["robots"];
    if (!robots.isArray())
    {
        cerr << "Error: 'robots' is not an array." << endl;
        return 1;
    }

    for (const auto &robot : robots)
    {
        if (!robot.isObject())
        {
            cerr << "Error: Each robot should be an object." << endl;
            continue;
        }

        string type = robot["id"].asString();
        cout << "Loaded Robot: " << type << endl;
    }



    return 0;
}