#pragma once

#include "AMPCore.h"
#include "CollisionDetector.h"
#include "MyAStar.h"
#include "KDTree.hpp"

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
            };

            /// @brief Solve a motion planning problem.
            /// @param problem Multi-agent motion planning problem
            /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
            amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem);

            DecentralizedMultiAgentPRMStar(const unsigned int& n, const unsigned int& d, const double& mu, const double& zeta) 
                                           : nSamples(n), d(d), mu_free(mu), zeta_d(zeta) {}
            ~DecentralizedMultiAgentPRMStar() = default;

        private:
            void sampleEnv(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result);
            void createGraph(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result);
            bool checkEdgeCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& current, const Eigen::Vector2d& neighbor);
            bool checkObstacleCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& config);
            bool checkRobotCollision(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const Eigen::Vector2d& q_near, const Eigen::Vector2d& q_new, const int& agentNum, const int& agentNumHighPri, const int& timeStep);
            bool highPriGoalCollision(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const int& agentNum, int& agentCollisionNum);

            unsigned int nSamples{0};
            unsigned int d{2};
            double mu_free{336.0};
            double zeta_d{M_PI};
    };
}
