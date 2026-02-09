#include "globals.h"
#include <iostream>


void playSound(const std::string &sound_path) {
    std::string command = "ffplay -autoexit -nodisp " + sound_path + " &";
    system(command.c_str());
}