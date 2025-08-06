#include "Dace-Template/odometry/odom.hpp"
#include "Dace-Template/drive/exit_conditions.hpp"
#include "Dace-Template/PID/pid.hpp"
#include <cmath>
#include "pros/rtos.hpp"

namespace dace {

constexpr double TICKS_PER_REV = 360.0;
constexpr double PI = 3.141592653589793;

Odometry* odom = nullptr;

Odometry::Odometry(int vert1, pros::Rotation* vert2, int horiz1, pros::Rotation* horiz2, double wheelDiameter)
    : vertical1(nullptr), vertical2(vert2), horizontal1(nullptr), horizontal2(horiz2),
      x(0), y(0), theta(0), lastVert(0), lastHoriz(0), linkedDrive(nullptr), trackingWheelDiam(wheelDiameter) {

    ticksToInches = (wheelDiameter * PI) / TICKS_PER_REV;

    if (vert1 != -1) vertical1 = new pros::Rotation(vert1);
    if (horiz1 != -1) horizontal1 = new pros::Rotation(horiz1);

    if (vertical1) vertical1->reset_position();
    if (vertical2) vertical2->reset_position();
    if (horizontal1) horizontal1->reset_position();
    if (horizontal2) horizontal2->reset_position();

    motionTask = new pros::Task{taskEntry, this};
}

void Odometry::linkDrive(Drive* drive) {
    linkedDrive = drive;
}

void Odometry::reset() {
    x = 0;
    y = 0;
    theta = 0;
    lastVert = 0;
    lastHoriz = 0;

    if (vertical1) vertical1->reset_position();
    if (vertical2) vertical2->reset_position();
    if (horizontal1) horizontal1->reset_position();
    if (horizontal2) horizontal2->reset_position();
}

void Odometry::update() {
    double vertTicks = (vertical1 ? vertical1->get_position() : 0);
    double horizTicks = (horizontal1 ? horizontal1->get_position() : 0);

    double dVert = (vertTicks - lastVert) * ticksToInches;
    double dHoriz = (horizTicks - lastHoriz) * ticksToInches;

    lastVert = vertTicks;
    lastHoriz = horizTicks;

    theta = linkedDrive ? linkedDrive->getIMU().get_heading() : 0;

    double rad = theta * PI / 180.0;
    x += dHoriz * std::cos(rad);
    y += dVert * std::sin(rad);
}

double Odometry::getX() const { return x; }
double Odometry::getY() const { return y; }
double Odometry::getTheta() const { return theta; }

void Odometry::wait() {
    while (isRunning) pros::delay(10);
}

// Async entry points with speed parameter
void Odometry::turnToHeading(double heading, SpeedSettings speed) {
    currentMotion = MotionType::Turn;
    param1 = heading;
    motionSpeed = speed;
    isRunning = true;
}

void Odometry::driveToX(double xTarget, bool back, SpeedSettings speed) {
    currentMotion = MotionType::DriveX;
    param1 = xTarget;
    backwards = back;
    motionSpeed = speed;
    isRunning = true;
}

void Odometry::driveToY(double yTarget, bool back, SpeedSettings speed) {
    currentMotion = MotionType::DriveY;
    param1 = yTarget;
    backwards = back;
    motionSpeed = speed;
    isRunning = true;
}

void Odometry::moveToPos(double xTarget, double yTarget, double finalHeading, bool back, SpeedSettings speed) {
    currentMotion = MotionType::MoveTo;
    param1 = xTarget;
    param2 = yTarget;
    param3 = finalHeading;
    backwards = back;
    motionSpeed = speed;
    isRunning = true;
}

// -------- Internal Async Logic --------

void Odometry::taskEntry(void* instance) {
    static_cast<Odometry*>(instance)->motionLoop();
}

void Odometry::motionLoop() {
    while (true) {
        pros::delay(10);
        update();

        if (!isRunning || !linkedDrive) continue;

        auto clamp = [](double val, int min, int max) {
            return std::fmax(min, std::fmin(max, val));
        };

        switch (currentMotion) {
            case MotionType::Turn: {
                PID turnPID(1.2, 0.0, 0.3);
                turnPID.setHeadingMode(true);
                turnPID.setTarget(param1);

                while (!turnPID.isSettled()) {
                    update();
                    double power = turnPID.calculate(linkedDrive->getIMU().get_heading());
                    power = clamp(power, motionSpeed.minSpeed, motionSpeed.maxSpeed);
                    linkedDrive->setTank(-power, power);
                    pros::delay(10);
                }
                break;
            }

            case MotionType::DriveX: {
                double heading = (param1 > x) ? 0 : 180;
                if (backwards) heading = std::fmod(heading + 180, 360);
                turnToHeading(heading, motionSpeed);
                wait();

                PID drivePID(0.5, 0.0, 0.05);
                drivePID.setTarget(param1);

                while (!drivePID.isSettled()) {
                    update();
                    double power = drivePID.calculate(x);
                    power = clamp(power, motionSpeed.minSpeed, motionSpeed.maxSpeed);
                    if (backwards) power *= -1;
                    linkedDrive->setTank(power, power);
                    pros::delay(10);
                }
                break;
            }

            case MotionType::DriveY: {
                double heading = (param1 > y) ? 90 : 270;
                if (backwards) heading = std::fmod(heading + 180, 360);
                turnToHeading(heading, motionSpeed);
                wait();

                PID drivePID(0.5, 0.0, 0.05);
                drivePID.setTarget(param1);

                while (!drivePID.isSettled()) {
                    update();
                    double power = drivePID.calculate(y);
                    power = clamp(power, motionSpeed.minSpeed, motionSpeed.maxSpeed);
                    if (backwards) power *= -1;
                    linkedDrive->setTank(power, power);
                    pros::delay(10);
                }
                break;
            }

            case MotionType::MoveTo: {
                update();
                double dx = param1 - x;
                double dy = param2 - y;
                double totalDist = std::sqrt(dx * dx + dy * dy);

                PID drivePID(0.6, 0.0, 0.05);
                PID turnPID(2.0, 0.0, 0.3);
                turnPID.setHeadingMode(true);
                drivePID.setTarget(totalDist);

                double startX = x;
                double startY = y;

                while (!drivePID.isSettled()) {
                    update();

                    double dx = param1 - x;
                    double dy = param2 - y;
                    double pathHeading = std::atan2(dy, dx) * 180.0 / PI;
                    if (pathHeading < 0) pathHeading += 360.0;
                    if (backwards) pathHeading = std::fmod(pathHeading + 180.0, 360.0);

                    double currentDist = std::sqrt((x - startX) * (x - startX) + (y - startY) * (y - startY));
                    double forward = drivePID.calculate(currentDist);
                    forward = clamp(forward, motionSpeed.minSpeed, motionSpeed.maxSpeed);
                    if (backwards) forward *= -1;

                    double turn = turnPID.calculate(theta - pathHeading);
                    double left = forward - turn;
                    double right = forward + turn;

                    linkedDrive->setTank(left, right);
                    pros::delay(10);
                }

                linkedDrive->setTank(0, 0);

                if (param3 >= 0 && param3 <= 360) {
                    turnToHeading(param3, motionSpeed);
                    wait();
                }
                break;
            }

            default: break;
        }

        linkedDrive->setTank(0, 0);
        currentMotion = MotionType::None;
        isRunning = false;
    }
}

} // namespace dace
