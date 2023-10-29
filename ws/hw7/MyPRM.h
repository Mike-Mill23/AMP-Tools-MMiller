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
    class MyPRM : public PRM2D {
        public:
            struct PRMResult {
                std::shared_ptr<Graph<double>> roadmap{};
                std::unique_ptr<pointVec> sampledPoints{};
                amp::Path2D path{};
                double pathLength{};
                double compTime{};

                Eigen::Vector2d operator()(amp::Node& node) const { return Eigen::Vector2d(sampledPoints->at(node)[0], sampledPoints->at(node)[1]); }
            };

            MyPRM(const unsigned int& n, const double& r, const bool& pathSmoothing) : nSamples(n), neighborRadius(r), pathSmoothing(pathSmoothing) {}
            ~MyPRM() = default;

            /// @brief Solve a motion planning problem.
            /// @param problem Motion planning problem
            /// @return Path solution of the point agent
            amp::Path2D plan(const amp::Problem2D& problem);

            int numValidSolutions{0};
            std::vector<double> pathLengthDataSet{};
            std::vector<double> compTimeDataSet{};

        private:
            void sampleEnv(const amp::Problem2D& problem, MyPRM::PRMResult& result);
            void createGraph(const amp::Problem2D& problem, MyPRM::PRMResult& result);
            void smoothPath(const amp::Problem2D& problem, MyPRM::PRMResult& result);

            unsigned int nSamples{0};
            double neighborRadius{1.0};
            bool pathSmoothing{false};
    };
}
