#pragma once
#include "main.h"

namespace dace {

    //Piston example
    inline Piston clamp('A');
   
    void set_clamp(bool input);
    void clamp_opcontrol();

    //Intake example
    inline pros::Motor intake(10);

    void set_intake(int input);
    void intake_opcontrol();
}//namespace dace

