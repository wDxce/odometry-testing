#include "Dace-Template/drive/exit_conditions.hpp"
#include "pros/rtos.hpp"
#include <cmath>

/*
Exit Conditions
*/

namespace dace{

    // Global instance of ExitConditions
    ExitConditions exitConfig;

    void setExitConfig(double angleTol, double distTol, int timeoutMs, double xTol, double yTol) {
        exitConfig = ExitConditions(angleTol, distTol, timeoutMs, xTol, yTol);
    }

    ExitConditions::ExitConditions(double angleTolerance,
                                   double distanceTolerance,
                                   int timeoutMs,
                                   double xTolerance,
                                   double yTolerance)
        : angleTol(angleTolerance), distTol(distanceTolerance),
          timeout(timeoutMs), xTol(xTolerance), yTol(yTolerance),
          targetX(0), targetY(0), targetTheta(0), startTime(0) {}

    void ExitConditions::setTarget(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetTheta = heading;
        startTime = pros::millis();
    }

    void ExitConditions::reset() {
        startTime = pros::millis();
    }
    
    bool ExitConditions::shouldExit(double currentX, double currentY, double currentHeading, bool pidSettled) {
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        double distError = std::sqrt(dx * dx + dy * dy);
        double angleError = std::fmod((targetTheta - currentHeading + 540.0), 360.0) - 180.0;
    
        bool distMet = distError <= distTol;
        bool angleMet = std::abs(angleError) <= angleTol;
        bool xMet = std::abs(targetX - currentX) <= xTol;
        bool yMet = std::abs(targetY - currentY) <= yTol;
        bool timeoutMet = (pros::millis() - startTime) >= timeout;
    
        return pidSettled || (distMet && angleMet) || (xMet && yMet) || timeoutMet;
    }

}//namespace dace