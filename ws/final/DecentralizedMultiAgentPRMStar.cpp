# include "DecentralizedMultiAgentPRMStar.h"

using namespace amp;


amp::MultiAgentPath2D DecentralizedMultiAgentPRMStar::plan(const amp::MultiAgentProblem2D& problem) {
    DecentralizedMultiAgentPRMStarResult result{};
    result.sampledPoints = std::make_unique<std::vector<Eigen::Vector2d>>();
    result.roadmap = std::make_shared<amp::Graph<double>>();
    result.path.agent_paths.clear();

    sampleEnv(problem, result);
    createGraph(problem, result);

    for (int i = 0; i < problem.numAgents(); i++) {
        result.path.agent_paths.push_back(Path2D());

        ShortestPathProblem prmStarProblem{};
        prmStarProblem.graph = result.roadmap;
        prmStarProblem.init_node = 2 * i;
        prmStarProblem.goal_node = (2 * i) + 1;

        PRMStarSearchHeuristic heuristic{prmStarProblem, result.sampledPoints};

        MyAStar aStar{};
        MyAStar::GraphSearchResult nodePath = aStar.search(prmStarProblem, heuristic);

        if (nodePath.success) {
            for (auto n : nodePath.node_path) {
                result.path.agent_paths[i].waypoints.push_back(result(n));
            }
        } else {
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
        }
    }

    return result.path;
}

void DecentralizedMultiAgentPRMStar::sampleEnv(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result) {
    bool isCollision{true};
    Eigen::Vector2d sample{};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disx(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disy(problem.y_min, problem.y_max);

    for (int i = 0; i < nSamples; i++) {
        isCollision = true;

        while (isCollision) {
            sample << disx(gen), disy(gen);
            isCollision = checkObstacleCollisions(problem, sample);
        }

        result.sampledPoints->push_back(sample);
    }

    for (int i = 0; i < problem.agent_properties.size(); i++) {
        result.sampledPoints->insert(result.sampledPoints->begin() + (2 * i), problem.agent_properties[i].q_init);
        result.sampledPoints->insert(result.sampledPoints->begin() + ((2 * i) + 1), problem.agent_properties[i].q_goal);
    }

    return;
}

void DecentralizedMultiAgentPRMStar::createGraph(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result) {
    pointVec kdPoints{};
    for (auto eigVec : *(result.sampledPoints)) {
        point_t kdPoint{eigVec[0], eigVec[1]};
        kdPoints.push_back(kdPoint);
    }

    bool isCollision{false};
    KDTree mapTree{kdPoints};
    double gammaPRM = (2 * pow(1 + (1/d), 1/d) * pow(mu_free/zeta_d, 1/d)) + 0.001;
    double prmStarRad = gammaPRM * pow(log(nSamples)/nSamples, 1/d);

    for (int i = 0; i < result.sampledPoints->size(); i++) {
        Eigen::Vector2d currentSampleEig{result.sampledPoints->at(i)};
        point_t currentSampleVec{currentSampleEig[0], currentSampleEig[1]};
        pointVec neighbors = mapTree.neighborhood_points(currentSampleVec, prmStarRad);

        for (auto neighbor : neighbors) {
            Eigen::Vector2d neighborEig{neighbor[0], neighbor[1]};
            isCollision = checkEdgeCollisions(problem, currentSampleEig, neighborEig);

            if (!isCollision) {
                double nodeDist = distanceL2(currentSampleEig, neighborEig);
                uint32_t nodeIndex = static_cast<Node>(i);
                uint32_t neighborNodeIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), neighborEig) - result.sampledPoints->begin());
                result.roadmap->connect(nodeIndex, neighborNodeIndex, nodeDist);
            }
        }
    }

    return;
}

bool DecentralizedMultiAgentPRMStar::checkEdgeCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& current, const Eigen::Vector2d& neighbor) {
    int numSubdivisions{6};
    Eigen::Vector2d direction = neighbor - current;

    for (auto obstacle : problem.obstacles) {
        Eigen::Vector2d c{0.0, 0.0};
        double distToObs = d_iq(neighbor, obstacle, c);

        for (auto agent : problem.agent_properties) {
            if (distToObs <= distanceL2(neighbor, current) + agent.radius) {
                for (int m = 0; m < numSubdivisions; m++) {
                    int numPoints = pow(2, m);
                    double fraction = 1.0 / (2 * numPoints);

                    for (int n = 0; n < numPoints; n++) {
                        Eigen::Vector2d subPoint = current + (direction * (fraction + (2 * n * fraction)));
                        distToObs = d_iq(subPoint, obstacle, c);

                        if (distToObs <= agent.radius) {
                            return true;
                        }
                    }
                }
            }
        }
    }

    return false;
}

bool DecentralizedMultiAgentPRMStar::checkObstacleCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& config) {
    for (auto obstacle : problem.obstacles) {
        Eigen::Vector2d c{0.0, 0.0};
        double distToObs = d_iq(config, obstacle, c);

        for (auto agent : problem.agent_properties) {
            if (distToObs <= agent.radius) {
                return true;
            }
        }
    }

    return false;
}

bool DecentralizedMultiAgentPRMStar::checkRobotCollision(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const Eigen::Vector2d& q_near, const Eigen::Vector2d& q_new, const int& agentNum, const int& agentNumHighPri, const int& timeStep) {
    int numSubdivisions{6};
    Eigen::Vector2d direction = q_new - q_near;
    Eigen::Vector2d q_new_other{};
    Eigen::Vector2d q_near_other{};
    
    if (timeStep > result.path.agent_paths[agentNumHighPri].waypoints.size()) {
        q_new_other = result.path.agent_paths[agentNumHighPri].waypoints[result.path.agent_paths[agentNumHighPri].waypoints.size() - 1];
        q_near_other = q_new_other;
    } else {
        q_new_other = result.path.agent_paths[agentNumHighPri].waypoints[timeStep];
        q_near_other = result.path.agent_paths[agentNumHighPri].waypoints[timeStep - 1];
    }
    Eigen::Vector2d direction_other = q_new_other - q_near_other;

    double sumRadii = problem.agent_properties[agentNum].radius + problem.agent_properties[agentNumHighPri].radius;
    double agentCenterDist = distanceL2(q_new, q_new_other);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    agentCenterDist = distanceL2(q_near, q_new_other);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    agentCenterDist = distanceL2(q_near_other, q_new);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    if ((distanceL2(q_near, q_new_other) <= distanceL2(q_new, q_near) + sumRadii) || (distanceL2(q_near_other, q_new) <= distanceL2(q_new_other, q_near_other) + sumRadii)) {
        for (int m = 0; m < numSubdivisions; m++) {
            int numPoints = pow(2, m);
            double fraction = 1.0 / (2 * numPoints);
            for (int n = 0; n < numPoints; n++) {
                Eigen::Vector2d subPoint = q_near + (direction * (fraction + (2 * n * fraction)));
                Eigen::Vector2d subPoint_other = q_near_other + (direction_other * (fraction + (2 * n * fraction)));
                agentCenterDist = distanceL2(subPoint, subPoint_other);
                if (agentCenterDist <= 1.1 * sumRadii) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool DecentralizedMultiAgentPRMStar::highPriGoalCollision(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const int& agentNum, int& agentCollisionNum) {
    int agentSize = result.path.agent_paths[agentNum].waypoints.size();

    for (int i = 0; i < agentNum; i++) {
        int agentHighPriSize = result.path.agent_paths[i].waypoints.size();
        if (agentSize < agentHighPriSize) {
            Eigen::Vector2d q_new = result.path.agent_paths[agentNum].waypoints[agentSize - 1];
            Eigen::Vector2d q_near = q_new;

            for (int j = agentSize - 1; j < agentHighPriSize; j++) {
                if (checkRobotCollision(problem, result, q_near, q_new, agentNum, i, j)) {
                    agentCollisionNum = i;
                    return true;
                }
            }
        }
    }

    return false;
}
