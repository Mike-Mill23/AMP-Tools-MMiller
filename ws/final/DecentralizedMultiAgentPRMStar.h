#pragma once

#include "AMPCore.h"
#include "hw/HW8.h"
#include "CollisionDetector.h"
#include "MyAStar.h"
#include "KDTree.hpp"  // https://github.com/crvs/KDTree
#include "Hungarian.h" // https://github.com/mcximing/hungarian-algorithm-cpp

#include <random>
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class DecentralizedMultiAgentPRMStar : public MultiAgentCircleMotionPlanner2D {
        public:
            struct DecentralizedMultiAgentPRMStarResult {
                std::shared_ptr<Graph<double>> roadmap{};
                std::unique_ptr<std::vector<Eigen::Vector2d>> sampledPoints{};
                amp::MultiAgentPath2D path{};

                Eigen::Vector2d operator()(amp::Node& node) const { return sampledPoints->at(node); }

                DecentralizedMultiAgentPRMStarResult() = default;
                DecentralizedMultiAgentPRMStarResult(uint32_t numAgents) { path = MultiAgentPath2D(numAgents); }
            };

            /// @brief Solve a motion planning problem.
            /// @param problem Multi-agent motion planning problem
            /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
            amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& initProblem);

            DecentralizedMultiAgentPRMStar(const unsigned int& n, const unsigned int& d, const double& mu, const double& zeta, const unsigned int& numTasks) 
                                           : nSamples(n), d(d), mu_free(mu), zeta_d(zeta), numTasks(numTasks) {}
            ~DecentralizedMultiAgentPRMStar() = default;

            double compTime{0.0};
            double pathLength{0.0};

        private:
            void sampleEnv(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result);
            void createGraph(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result);
            void assignTasks(amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result);
            void decentralizePath(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const int& agentNum);
            bool recursiveBackup(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const int& agentNum, const std::vector<Eigen::Vector2d>& path, int numBackups, Eigen::Vector2d& firstCollisionPoint);
            bool checkEdgeCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& current, const Eigen::Vector2d& neighbor);
            bool checkObstacleCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& config);
            bool checkRobotCollision(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const Eigen::Vector2d& curr, const Eigen::Vector2d& next, const int& agentNum, const int& hiPriAgentNum, const int& timeStep);

            unsigned int nSamples{0};
            unsigned int d{2};
            double mu_free{336.0};
            double zeta_d{M_PI};
            unsigned int numTasks{1};
    };
}
