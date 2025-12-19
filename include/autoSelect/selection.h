#pragma once

#include <string>

// selector configuration
#define HUE 360
#define AUTONS "3 Ball", "6 Ball (Don't Use)", "Do Nothing"
#define DEFAULT 1

namespace selector
{

    extern int auton;
    const char *b[] = {AUTONS, ""};
    void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
