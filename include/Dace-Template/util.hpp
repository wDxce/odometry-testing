#pragma once
#include "main.h"

extern dace::Drive chassis;
extern dace::Odometry trackers;
// Controller
// The master controller object is used to receive input from the user via joysticks and buttons.
extern pros::Controller master;

/*
Util Header File
*/

namespace dace{

    void odomStandards();  // links odometry to chassis and sets exit conditions
    void pidStandards();   // sets exit conditions without odometry

    class util{

        public:
            
        private:

    };

}//namespace dace