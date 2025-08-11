#include "main.h"


dace::Drive chassis(

	//Left side of chassis motors
	{1,2,3},
	{3,4,5},

	7,//imu port
	2.75,//wheel diameter in inches
	450//Wheel RPM = cartridge * (motor gear / wheel gear)

);

//comment out if not using tracking wheels
dace::Odometry trackers(

	1,//vertical tracking wheel 1
	nullptr,//vertical tracker 2 if not 2 just type nullptr
	2,//horizontal tracker 1
	nullptr, //horizontal tracker 2 if not 2 just type nullptr
	2.75 //tracking wheel diameter
	
);


void initialize() {
 	// Set PID values for all movement types
    dace::PID_Values(
        0.0, 0.0, 0.0,   // drive:  P, I, D
        0.0, 0.0, 0.0,  // turn:   P, I, D
        0.0, 0.0, 0.0,  // swing:  P, I, D
        0.0, 0.0, 0.0   // curve:  P, I, D
    );

    // Optional: Link odometry to chassis
    trackers.linkDrive(&chassis);

    // Optional: Set brake mode
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

}


void disabled() {}

void competition_initialize() {}


void autonomous() {
	//Skills
	//left-Side
}


void opcontrol() {

	while (true) {

		//Pick a driving style

		/*
		the number in the () is the deadzone 10 is an ideal number
		*/
	
		//Uses left analog Y for fwd/rev and right analog X for lateral motion
		dace::arcade_drive(10);

		//Uses left analog Y to control the left side of chassis
		//Uses right Analog Y to control right side of chassis
		//Push both up/down at same time to go fwd/rev
		//dace::tank_drive(10);

		//Uses one sitck for fwd/rev (Y) and lateral motion (X)
		//dace::single_stick_drive(dace::Stick::Right, 10);
		//dace::single_stick_drive(dace::Stick::Left, 10);

		//Examples of subsystems
		dace::clamp_opcontrol();
		dace::intake_opcontrol();
		pros::delay(10);
	}
}