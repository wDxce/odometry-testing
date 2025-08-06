#pragma once
#include <vector>
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp" // Correct include for pros::Task

/*
Drive Header File
*/

namespace swing {
    enum Direction { Left, Right };
}

namespace dace {

    class Drive {
    public:
        Drive(const std::vector<int>& leftPorts,
              const std::vector<int>& rightPorts,
              int imuPort,
              double wheelDiameter,
              int wheelRPM);

        pros::Imu& getIMU();
        void setBrakeMode(pros::motor_brake_mode_e mode);
        void setTank(double leftVoltage, double rightVoltage);

        void drive(double distance_in, int drive_speed);                // async drive
        void turn_to_heading(double theta, int turn_speed);            // async turn
        void swing(swing::Direction dir, double theta, int speed);     // async swing
        void curve(double rightSpeed, double leftSpeed, double theta); // async curve

        void wait(); // blocks until async motion finishes

    private:
        std::vector<pros::Motor> leftMotors;
        std::vector<pros::Motor> rightMotors;
        pros::Imu imu;

        double wheelDiam;
        int rpm;

        pros::Task* motionTask = nullptr;
        bool isRunning = false;

        // Motion type enum
        enum class MotionType { None, Drive, Turn, Swing, Curve };
        MotionType currentMotion = MotionType::None;

        // Motion parameters
        double param1 = 0;  // distance_in or targetHeading or rightSpeed
        double param2 = 0;  // drive_speed or turn_speed or leftSpeed
        int intParam = 0;   // optional int parameter (e.g. swing direction encoded)
        swing::Direction swingDir;

        void motionLoop(); // task callback
        static void taskEntry(void* ptr); // static wrapper for task callback
    };

} // namespace dace
