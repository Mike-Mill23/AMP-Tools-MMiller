#pragma once

#include "AMPCore.h"
#include "hw/HW8.h"
#include "CollisionDetector.h"

#include <random>
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyCentralizedMultiAgentRRT : public CentralizedMultiAgentRRT {
        public:
            struct CentralizedMultiAgentRRTResult {
                std::shared_ptr<Graph<double>> roadmap{};
                std::unique_ptr<std::vector<Eigen::VectorXd>> sampledPoints{};
                amp::MultiAgentPath2D path{};
                double treeSize{};
                double compTime{};

                Eigen::VectorXd operator()(amp::Node& node) const { return sampledPoints->at(node); }
            };

            /// @brief Solve a motion planning problem.
            /// @param problem Multi-agent motion planning problem
            /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
            amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem);

            MyCentralizedMultiAgentRRT(const unsigned int& n, const double& r, const double& goalProb, const double& epsilon) 
                                       : nSamples(n), stepSize(r), goalProb(goalProb), epsilon(epsilon) {}
            ~MyCentralizedMultiAgentRRT() = default;

            int numValidSolutions{0};
            int resultsFound{0};
            std::vector<double> treeSizeDataSet{};
            std::vector<double> compTimeDataSet{};

        private:
            void createTree(const amp::MultiAgentProblem2D& problem, MyCentralizedMultiAgentRRT::CentralizedMultiAgentRRTResult& result, bool& resultFound);
            bool checkObstacleCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_new, const int& agentNum);
            bool checkRobotCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_new, const int& agentNum);

            unsigned int nSamples{0};
            double stepSize{1.0};
            double goalProb{0.05};
            double epsilon{0.25};
            int planNum{0};
    };
}
