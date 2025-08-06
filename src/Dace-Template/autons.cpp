#include "main.h"

/*
For odometry if setting custom speed you must call the bool for forwards or backwards.
*/

void odomTest(){
    dace::odomStandards(); //needed for all autons involving odometry
    
    pros::delay(100);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD); //sets the brake type

    trackers.moveToPos(10,10,45, false, {127, 85}); //x, y, theta, fwd/rev, {maxSpeed, minSpeed}

    chassis.wait();// waits for all functions to finish before moving on to next task

    trackers.driveToX(15, true); //x, backwards set true using default speeds

    chassis.wait();

    trackers.moveToPos(-10,-10, 180, true); //x, y, theta, backwards set true using default speeds
    
}

void pidTest(){
    dace::pidStandards(); //needed for all autons using pid movements

    chassis.drive(24,110); //distance in inches, speed

}