#include "loco.h"
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include "globals.h"


Loco::Loco(const std::string &filename) {
    std::string full_path = std::string(LOCO_CONFIG_DIR) + "/" + filename;
    YAML::Node config = YAML::LoadFile(full_path);

    name = config["name"].as<std::string>();
    m_start_throttle = config["start_throttle"].as<int>();
    m_stop_throttle = config["stop_throttle"].as<int>();
    m_cruise_throttle = config["cruise_throttle"].as<int>();

    for (YAML::const_iterator it = config["images"].begin(); it != config["images"].end(); ++it) {
        images[it->first.as<std::string>()] = it->second.as<std::string>();
    }

    for (YAML::const_iterator it = config["sounds"].begin(); it != config["sounds"].end(); ++it) {
        Sound s;
        s.path = std::string(SOUND_DIR) + "/" + it->second["path"].as<std::string>();
        s.duration = it->second["duration"].as<int>();
        sounds[it->first.as<std::string>()] = s;
    }

}




Sound Loco::getSound(const std::string &sound_name) const {
    auto it = sounds.find(sound_name);
    if (it != sounds.end())
        return it->second;
    return Sound{"", 0};  // fallback
}

