#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW4.h"
#include "CollisionDetector.h"
#include "MyGridCSpace.h"

#include <queue>
#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyManipulatorWaveFrontAlgorithm : public ManipulatorWaveFrontAlgorithm {
        public:
            /// @brief Construct the algorithm class by providing a C-space constructor member (from HW4). 
            /// @param c_space_constructor Shared pointer to a C-space constructor object. 
            MyManipulatorWaveFrontAlgorithm(const std::shared_ptr<GridCSpace2DConstructor>& c_space_constructor) : ManipulatorWaveFrontAlgorithm(c_space_constructor) {}

            /******* User Implemented Methods ********/

            /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
            /// @param grid_cspace Your grid discretization C-space from HW4. 
            /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
            /// NOTE: For Exercise 2), You can use 
            /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
            amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace);

            /*****************************************/
    };
}
