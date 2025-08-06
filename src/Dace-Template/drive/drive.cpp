#include "Dace-Template/drive/drive.hpp"
#include "Dace-Template/PID/pid.hpp"
#include <cmath>

constexpr double PI = 3.141592653589793;

// Clamp for C++11
template <typename T>
T clamp(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}

namespace dace {

Drive::Drive(const std::vector<int>& leftPorts,
             const std::vector<int>& rightPorts,
             int imuPort,
             double wheelDiameter,
             int wheelRPM)
    : imu(imuPort), wheelDiam(wheelDiameter), rpm(wheelRPM) {

    for (int port : leftPorts)
        leftMotors.emplace_back(port);

    for (int port : rightPorts)
        rightMotors.emplace_back(port);

    imu.reset();
    while (imu.is_calibrating()) pros::delay(10);
}

void Drive::setBrakeMode(pros::motor_brake_mode_e mode) {
    for (auto& motor : leftMotors)
        motor.set_brake_mode(mode);
    for (auto& motor : rightMotors)
        motor.set_brake_mode(mode);
}

pros::Imu& Drive::getIMU() {
    return imu;
}

void Drive::setTank(double leftVoltage, double rightVoltage) {
    for (auto& motor : leftMotors)
        motor.move_voltage(leftVoltage * 120);  // ±100 → ±12000mV
    for (auto& motor : rightMotors)
        motor.move_voltage(rightVoltage * 120);
}

void Drive::drive(double distance_in, int drive_speed) {
    if (isRunning) wait();

    param1 = distance_in;
    param2 = drive_speed;
    currentMotion = MotionType::Drive;
    isRunning = true;
    motionTask = new pros::Task(taskEntry, this);
}

void Drive::turn_to_heading(double theta, int turn_speed) {
    if (isRunning) wait();

    param1 = theta;
    param2 = turn_speed;
    currentMotion = MotionType::Turn;
    isRunning = true;
    motionTask = new pros::Task(taskEntry, this);
}

void Drive::swing(swing::Direction dir, double theta, int speed) {
    if (isRunning) wait();

    swingDir = dir;
    param1 = theta;
    param2 = speed;
    currentMotion = MotionType::Swing;
    isRunning = true;
    motionTask = new pros::Task(taskEntry, this);
}

void Drive::curve(double rightSpeed, double leftSpeed, double theta) {
    if (isRunning) wait();

    param1 = rightSpeed;
    param2 = leftSpeed;
    intParam = theta;
    currentMotion = MotionType::Curve;
    isRunning = true;
    motionTask = new pros::Task(taskEntry, this);
}

void Drive::wait() {
    if (motionTask) {
        while (isRunning) pros::delay(10);
        delete motionTask;
        motionTask = nullptr;
    }
}

// Static entry point for the task
void Drive::taskEntry(void* ptr) {
    static_cast<Drive*>(ptr)->motionLoop();
}

void Drive::motionLoop() {
    switch (currentMotion) {
        case MotionType::Drive: {
            const double TICKS_PER_REV = 360.0;
            const double CIRC = wheelDiam * PI;
            const double TICKS_PER_INCH = TICKS_PER_REV / CIRC;

            int targetTicks = param1 * TICKS_PER_INCH;

            for (auto& m : leftMotors) m.tare_position();
            for (auto& m : rightMotors) m.tare_position();

            PID drivePID(drivePIDVals.kP, drivePIDVals.kI, drivePIDVals.kD);
            drivePID.setTarget(targetTicks);

            while (!drivePID.isSettled()) {
                int currentTicks = (leftMotors[0].get_position() + rightMotors[0].get_position()) / 2;
                double output = drivePID.calculate(currentTicks);

                output = clamp(output,
                    -static_cast<double>(std::abs(param2)),
                     static_cast<double>(std::abs(param2))
                );

                output *= (param1 < 0) ? -1 : 1;
                setTank(output, output);
                pros::delay(10);
            }

            break;
        }

        case MotionType::Turn: {
            PID turnPID(turnPIDVals.kP, turnPIDVals.kI, turnPIDVals.kD);
            turnPID.setHeadingMode(true);
            turnPID.setTarget(param1);

            while (!turnPID.isSettled()) {
                double power = turnPID.calculate(imu.get_heading());

                power = clamp(power,
                    -static_cast<double>(param2),
                     static_cast<double>(param2)
                );

                setTank(-power, power);
                pros::delay(10);
            }

            break;
        }

        case MotionType::Swing: {
            PID swingPID(swingPIDVals.kP, swingPIDVals.kI, swingPIDVals.kD);
            swingPID.setHeadingMode(true);
            swingPID.setTarget(param1);

            while (!swingPID.isSettled()) {
                double power = swingPID.calculate(imu.get_heading());

                power = clamp(power,
                    -static_cast<double>(param2),
                     static_cast<double>(param2)
                );

                if (swingDir == swing::Left)
                    setTank(0, power);
                else
                    setTank(-power, 0);

                pros::delay(10);
            }

            break;
        }

        case MotionType::Curve: {
            PID curvePID(curvePIDVals.kP, curvePIDVals.kI, curvePIDVals.kD);
            curvePID.setHeadingMode(true);
            curvePID.setTarget(intParam);

            while (!curvePID.isSettled()) {
                double heading = imu.get_heading();
                double error = curvePID.calculate(heading);

                double adjustedLeft = param2 - error;
                double adjustedRight = param1 + error;

                setTank(adjustedLeft, adjustedRight);
                pros::delay(10);
            }

            break;
        }

        default:
            break;
    }

    setTank(0, 0);
    isRunning = false;
    currentMotion = MotionType::None;
}

} // namespace dace
