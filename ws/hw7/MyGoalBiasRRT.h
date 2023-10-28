#pragma once

#include "AMPCore.h"
// #include "hw/HW7.h"

#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyGoalBiasRRT : public PointMotionPlanner2D {
        public:
            MyGoalBiasRRT() = default;
            ~MyGoalBiasRRT() = default;

            /// @brief Solve a motion planning problem. Derive class and override this method
            /// @param problem Motion planning problem
            /// @return Path solution of the point agent
            amp::Path2D plan(const amp::Problem2D& problem);

        private:
            
    };
}
