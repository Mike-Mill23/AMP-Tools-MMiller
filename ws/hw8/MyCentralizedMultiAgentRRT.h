#pragma once

#include "AMPCore.h"
#include "hw/HW8.h"
#include "CollisionDetector.h"
#include "KDTree.hpp"
#include "MyAStar.h"

#include <random>
#include <string.h>
#include <stdio.h>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyCentralizedMultiAgentRRT : public CentralizedMultiAgentRRT {
        public:
            struct CentralizedMultiAgentRRTResult {
                std::shared_ptr<Graph<double>> roadmap{};
                std::unique_ptr<pointVec> sampledPoints{};
                amp::MultiAgentPath2D path{};
                double treeSize{};
                double compTime{};

                Eigen::VectorXd operator()(amp::Node& node) const { return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(sampledPoints->at(node).data(), sampledPoints->at(node).size()); }
            };

            /// @brief Solve a motion planning problem.
            /// @param problem Multi-agent motion planning problem
            /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
            amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem);

            MyCentralizedMultiAgentRRT(const unsigned int& n, const double& r, const double& goalProb, const double& epsilon) 
                                       : nSamples(n), stepSize(r), goalProb(goalProb), epsilon(epsilon) {}
            ~MyCentralizedMultiAgentRRT() = default;

            std::vector<double> treeSizeDataSet{};
            std::vector<double> compTimeDataSet{};

        private:
            void createTree(const amp::MultiAgentProblem2D& problem, MyCentralizedMultiAgentRRT::CentralizedMultiAgentRRTResult& result);

            unsigned int nSamples{0};
            double stepSize{1.0};
            double goalProb{0.05};
            double epsilon{0.25};
    };
}
