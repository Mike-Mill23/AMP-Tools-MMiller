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
    class MyPointWaveFrontAlgorithm : public PointWaveFrontAlgorithm {
        public:
            /******* User Implemented Methods ********/

        /// @brief Create a discretized planning space of an environment (point agent). If you abstracted your GridCSpace2DConstructor, you may be
        /// able to use that code here to construct a discretized C-space for a point agent.
        /// @param environment Workspace and/or C-space (point agent)
        /// @return Unique pointer to a GridCSpace2D object (see HW4)
        std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment);

        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
        amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace);

        /*****************************************/

        private:
            std::vector<std::pair<std::size_t, std::size_t>> getNeighbors(const std::pair<std::size_t, std::size_t>& cell, const std::size_t& numx0Cells, const std::size_t& numx1Cells);
    };
}
