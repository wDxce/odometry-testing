#include "main.h"

namespace dace {
    //Clamp example
    void set_clamp(bool input) { 
        clamp.set(input); 
    }

    void clamp_opcontrol() {
      clamp.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1));
    }

    //Intake example
    void set_intake(int input){
        intake.move(input);
    }

    void intake_opcontrol(){
        if (master.get_digital(DIGITAL_L1)){
            intake.move(127);
        } else if (master.get_digital(DIGITAL_L2)){
            intake.move(-127);
        } else {
            intake.move(0);
        }
    }

}