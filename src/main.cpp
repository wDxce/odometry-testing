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

		//Joystick deadzone value(recommend 10)
		const int DEADZONE = 10;

		// Get joystick values
		double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn    = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// Apply deadzone
		if (std::abs(forward) < DEADZONE) forward = 0;
		if (std::abs(turn) < DEADZONE) turn = 0;

		// Arcade drive
		double leftVoltage  = forward + turn;
		double rightVoltage = forward - turn;

		// Clamp to -127 || +127 voltage
		leftVoltage  = std::clamp(leftVoltage,  -127.0, 127.0);
		rightVoltage = std::clamp(rightVoltage, -127.0, 127.0);

		// Send voltage to motors
		chassis.setTank(leftVoltage, rightVoltage);

		dace::clamp_opcontrol();
		dace::intake_opcontrol();
		pros::delay(10);
	}
}