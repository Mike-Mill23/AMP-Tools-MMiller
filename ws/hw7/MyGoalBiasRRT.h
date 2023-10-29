#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "CollisionDetector.h"
#include "KDTree.hpp"
#include "MyAStar.h"

#include <random>
#include <string.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyGoalBiasRRT : public GoalBiasRRT2D {
        public:
            struct GoalBiasRRTResult {
                std::shared_ptr<Graph<double>> roadmap{};
                std::unique_ptr<pointVec> sampledPoints{};
                amp::Path2D path{};
                double pathLength{};
                double compTime{};

                Eigen::Vector2d operator()(amp::Node& node) const { return Eigen::Vector2d(sampledPoints->at(node)[0], sampledPoints->at(node)[1]); }
            };

            MyGoalBiasRRT(const unsigned int& n, const double& r, const double& goalProb, const double& epsilon, const bool& pathSmoothing) 
                         : nSamples(n), stepSize(r), goalProb(goalProb), epsilon(epsilon), pathSmoothing(pathSmoothing) {}
            ~MyGoalBiasRRT() = default;

            /// @brief Solve a motion planning problem. Derive class and override this method
            /// @param problem Motion planning problem
            /// @return Path solution of the point agent
            amp::Path2D plan(const amp::Problem2D& problem);

            int numValidSolutions{0};
            std::vector<double> pathLengthDataSet{};
            std::vector<double> compTimeDataSet{};

        private:
            void createTree(const amp::Problem2D& problem, MyGoalBiasRRT::GoalBiasRRTResult& result);
            void smoothPath(const amp::Problem2D& problem, MyGoalBiasRRT::GoalBiasRRTResult& result);

            unsigned int nSamples{0};
            double stepSize{1.0};
            double goalProb{0.05};
            double epsilon{0.25};
            bool pathSmoothing{false};
    };
}
