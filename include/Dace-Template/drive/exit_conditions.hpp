#pragma once

/*
Exit Conditions Header File
*/

namespace dace{

    class ExitConditions {
        public:
            ExitConditions(double angleTolerance = 1.0,
                           double distanceTolerance = 1.0,
                           int timeoutMs = 3000,
                           double xTolerance = 1.0,
                           double yTolerance = 1.0);

            void setTarget(double targetX, double targetY, double targetHeading);
            void reset();

            bool shouldExit(double currentX, double currentY, double currentHeading, bool pidSettled);

        private:
            double targetX, targetY, targetTheta;
            double angleTol, distTol, xTol, yTol;
            int timeout;
            int startTime;
    };

    // Global instance & config setter
    extern ExitConditions exitConfig;
    void setExitConfig(double angleTol, double distTol, int timeoutMs, double xTol, double yTol);

}//namespace dace