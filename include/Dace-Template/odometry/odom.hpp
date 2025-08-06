#pragma once
#include "pros/rotation.hpp"
#include "Dace-Template/drive/drive.hpp"
#include "pros/rtos.hpp"

namespace dace {

// Optional movement speed config struct
struct SpeedSettings {
    int maxSpeed = 90;
    int minSpeed = 90;
};

class Odometry {
  public:
    Odometry(int vert1, pros::Rotation* vert2, int horiz1, pros::Rotation* horiz2, double wheelDiamete = 2.75);

    void reset();
    void update();

    // Async movement functions (default to async behavior)
    void moveToPos(double xTarget, double yTarget, double finalHeading = -1, bool backwards = false, SpeedSettings speed = {});
    void driveToX(double xTarget, bool backwards = false, SpeedSettings speed = {});
    void driveToY(double yTarget, bool backwards = false, SpeedSettings speed = {});
    void turnToHeading(double heading, SpeedSettings speed = {});

    void wait();  // Waits for async motion to finish

    // Odometry accessors
    double getX() const;
    double getY() const;
    double getTheta() const;

    // Link drive for tank and IMU access
    void linkDrive(Drive* drive);

  private:
    pros::Rotation* vertical1;
    pros::Rotation* vertical2;
    pros::Rotation* horizontal1;
    pros::Rotation* horizontal2;

    double x, y, theta;
    double lastVert, lastHoriz;

    Drive* linkedDrive;

    double trackingWheelDiam;
    double ticksToInches;

    // Async motion thread
    pros::Task* motionTask = nullptr;
    bool isRunning = false;

    // Current movement type
    enum class MotionType { None, MoveTo, DriveX, DriveY, Turn };
    MotionType currentMotion = MotionType::None;

    // Parameters for motion
    double param1 = 0;      // x or heading or start
    double param2 = 0;      // y or not used
    double param3 = -1;     // final heading (optional)
    bool boolParam = false; // backwards = true/false
    bool backwards = false;
    SpeedSettings motionSpeed; // max/min voltages

    // Async motion runner
    void motionLoop();
    static void taskEntry(void* instance); // Static entry for pros::Task
};

extern Odometry* odom;

}  // namespace dace
