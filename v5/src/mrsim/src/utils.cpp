#include "mrsim/utils.h"

#include <json/json.h>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

namespace fs = std::filesystem;

// ---- internal helpers, not visible outside this file ----
namespace {
std::string lowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

bool loadJson(const std::string& path, Json::Value& root) {
    std::ifstream in(path, std::ifstream::binary);
    if (!in) {
        std::cerr << "Error opening JSON for reading: " << path << " — "
                  << std::strerror(errno) << "\n";
        return false;
    }
    Json::CharReaderBuilder b;
    std::string errs;
    if (!Json::parseFromStream(b, in, &root, &errs)) {
        std::cerr << "Error parsing JSON: " << path << " — " << errs << "\n";
        return false;
    }
    return true;
}

bool writeJsonAtomic(const std::string& path, const Json::Value& root) {
    const fs::path p(path);
    const fs::path tmp = p.parent_path() / (p.filename().string() + ".tmp");

    std::ofstream out(tmp, std::ofstream::binary | std::ofstream::trunc);
    if (!out) {
        std::cerr << "Error opening temp file for writing: " << tmp << " — "
                  << std::strerror(errno) << "\n";
        return false;
    }
    Json::StreamWriterBuilder w;
    w["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(w.newStreamWriter());
    writer->write(root, &out);
    out.close();

    std::error_code ec;
    fs::rename(tmp, p, ec);
    if (ec) {
        std::cerr << "Atomic rename failed: " << ec.message()
                  << " (dir may be read-only or not writable)\n";
        std::error_code ec2;
        fs::remove(tmp, ec2);
        return false;
    }
    return true;
}
} // namespace

// ---- public functions declared in utils.h ----

PlayerInfo selectOrCreatePlayer(const std::string& rankingPath) {
    Json::Value root;
    if (!loadJson(rankingPath, root)) {
        return {};
    }

    Json::Value& players = root["players"];
    if (!players.isArray()) {
        root["players"] = Json::Value(Json::arrayValue);
    }

    std::cout << "******************* WELCOME **********************\n";
    std::cout << "First choose your player :\n";
    for (Json::ArrayIndex i = 0; i < players.size(); ++i) {
        const Json::Value& p = players[i];
        std::string name = p.get("name", std::to_string(i + 1)).asString();
        std::cout << "\t-" << name << "\n";
    }

    std::cout << ">> ";
    std::string selected;
    std::getline(std::cin >> std::ws, selected);

    unsigned int max_id = 0;
    for (Json::ArrayIndex i = 0; i < players.size(); ++i) {
        const Json::Value& p = players[i];
        max_id = max(max_id, p.get("id", 0).asUInt());
        unsigned int id = p.get("id", 0).asUInt();
        string name =  p.get("name","").asString();
        int fav_lv = p.get("fav_level","").asInt();
        if (lowerCopy(p.get("name", "").asString()) == lowerCopy(selected)) {
            cout << "Welcome back, " << selected << "!\n";
            return PlayerInfo{ id, name, fav_lv };
        }
    }

    Json::Value new_player(Json::objectValue);
    new_player["id"] = max_id + 1;
    new_player["name"] = selected;
    new_player["fav_lev"] = 1;

    root["players"].append(new_player);

    if (!writeJsonAtomic(rankingPath, root)) {
        std::cerr << "Cannot write ranking file. "
                     "Check permissions or move it outside the source tree.\n";
        return {};
    }

    std::cout << "New player added: " << selected << "\n";
    int default_lv = 1;
    return PlayerInfo{max_id + 1, selected, default_lv};
}

bool saveMatchResult(const std::string& rankingPath,
                     const PlayerInfo& player,
                     double elapsed_seconds) {
    Json::Value root;
    if (!loadJson(rankingPath, root)) {
        return false;
    }

    if (!root["matches"].isArray()) {
        root["matches"] = Json::Value(Json::arrayValue);
    }

    unsigned int max_match_id = 0;
    const Json::Value& matches = root["matches"];
    for (Json::ArrayIndex i = 0; i < matches.size(); ++i) {
        max_match_id = std::max(max_match_id, matches[i].get("id", 0).asUInt());
    }

    Json::Value m(Json::objectValue);
    m["id"] = max_match_id + 1;
    m["player"] = player.name;
    m["player_id"] = player.id;
    m["time"] = elapsed_seconds;

    root["matches"].append(m);

    if (!writeJsonAtomic(rankingPath, root)) {
        std::cerr << "Failed to persist match result to " << rankingPath << "\n";
        return false;
    }
    return true;
}



