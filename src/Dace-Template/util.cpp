#include "main.h"

// Initialize the controller
// The master controller object allows for joystick and button input.
pros::Controller master(pros::E_CONTROLLER_MASTER);

/*
Util
*/

namespace dace{

    void odomStandards(){
        trackers.linkDrive(&chassis);
        trackers.reset();
        setExitConfig(1.0, 3.0, 3000, 1.0, 1.0);
    }

    void pidStandards(){
        setExitConfig(1.0, 3.0, 3000, 1.0, 1.0);
    }

}//namespace dace