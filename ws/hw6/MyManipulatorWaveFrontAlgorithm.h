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

            /// @param link_manipulator_agent Your link manipulator with forward and inverse kinematics implemented in HW4
            /// @param problem A planning problem with workspace obstacles and init/goal end effector locations
            /// @return A sequence of ManipulatorStates that takes the manipulator from the q_init location to q_goal
            amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
                ASSERT(link_manipulator_agent.nLinks() == 2, "Manipulator must have two links");
                wfProblem = problem;

                // Get the initial state from IK
                amp::ManipulatorState init_state = link_manipulator_agent.getConfigurationFromIK(problem.q_init);

                // Get the goal state from IK
                amp::ManipulatorState goal_state = link_manipulator_agent.getConfigurationFromIK(problem.q_goal);

                // Construct the grid cspace
                std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_c_space_constructor->construct(link_manipulator_agent, problem);

                // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
                // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
                // amp::Path2D path = planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
                // Visualizer::makeFigure(problem, link_manipulator_agent, path);
                // Visualizer::showFigures();
                // return path;
                return planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
            }

            /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
            /// @param grid_cspace Your grid discretization C-space from HW4. 
            /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
            /// NOTE: For Exercise 2), You can use 
            /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of representative points for each cell in the sequence.
            amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace);

            /*****************************************/

            private:
                std::vector<std::pair<std::size_t, std::size_t>> getNeighbors(const std::pair<std::size_t, std::size_t>& cell, const std::size_t& numx0Cells, const std::size_t& numx1Cells);
                std::pair<std::size_t, std::size_t> getInitCell(const amp::GridCSpace2D& grid_cspace, Eigen::Vector2d& q_init, const std::size_t& numx0Cells, const std::size_t& numx1Cells);
                std::pair<std::size_t, std::size_t> getGoalCell(const amp::GridCSpace2D& grid_cspace, Eigen::Vector2d& q_goal, const std::size_t& numx0Cells, const std::size_t& numx1Cells);

                amp::Problem2D wfProblem{};
    };
}
