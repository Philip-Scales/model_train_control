#ifndef LOCO_H
#define LOCO_H

#include "globals.h"
#include <map>
#include <string>

class decision;

class Loco {

protected:
    //TODO: change this
    //ptr used to access useful info to update agents from perception callbacks
    decision *m_decision_node_ptr;

public:
    std::string name;
    std::map<std::string, std::string> images;
    std::map<std::string, Sound> sounds;
    int m_start_throttle = 154;
    int m_stop_throttle = 138;
    int m_cruise_throttle = 170;

    //constructor
    // Reads loco information from a YAML file and initializes the Loco object.
    Loco(const std::string &filename);
    Sound getSound(const std::string &sound_name) const;


    //br78 range [0;1]: 0.60 0.64   153 168.3



};

#endif // LOCO_H

