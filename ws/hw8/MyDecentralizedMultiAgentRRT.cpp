# include "MyDecentralizedMultiAgentRRT.h"

using namespace amp;


amp::MultiAgentPath2D MyDecentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    // LOG("Plan Num: " << planNum);
    // planNum++;

    auto start = std::chrono::high_resolution_clock::now();
    DecentralizedMultiAgentRRTResult result{};
    result.sampledPoints = std::make_unique<std::vector<Eigen::Vector2d>>();
    result.roadmap = std::make_shared<amp::Graph<double>>();
    bool resultFound{false};
    int resultsFoundInternal{0};

    result.path.agent_paths.clear();
    for (int i = 0; i < problem.numAgents(); i++) {
        result.path.agent_paths.push_back(Path2D());

        createTree(problem, result, resultFound);

        if (resultFound) {
            resultsFound++;
            resultsFoundInternal++;
            Node parentNode = result.sampledPoints->size() - 1;
            while (1) {
                result.path.agent_paths[i].waypoints.insert(result.path.agent_paths[i].waypoints.begin(), result(parentNode));

                if (parentNode == 0) {
                    break;
                } else {
                    parentNode = result.roadmap->parents(parentNode)[0];
                }
            }

            int highPriAgentIndex{0};
            if (highPriGoalCollision(problem, result, i, highPriAgentIndex)) {
                int agentSize = result.path.agent_paths[i].waypoints.size();
                int highPriAgentSize = result.path.agent_paths[highPriAgentIndex].waypoints.size();
                int numPoints = highPriAgentSize - agentSize;
                Eigen::Vector2d stationaryPoint = result.path.agent_paths[i].waypoints[agentSize - 3];
                result.path.agent_paths[i].waypoints.insert(result.path.agent_paths[i].waypoints.begin() + (agentSize - 3), numPoints, stationaryPoint);
            }
        } else {
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
        }

        result.sampledPoints->clear();
        result.roadmap->clear();
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    result.compTime = static_cast<double>(duration.count()) * 1000;
    compTimeDataSet.push_back(result.compTime);

    // std::vector<std::vector<Eigen::Vector2d>> collisions{};
    // if (HW8::check(result.path, problem, collisions, false)) {
    //     numValidSolutions++;
    // } else {
    //     if ((resultsFoundInternal == problem.numAgents()) && collisions.size() > 0) {
    //         LOG("Path Collision!");
    //         Visualizer::makeFigure(problem, result.path, collisions);
    //         Visualizer::showFigures();
    //     }
    // }

    // Uncomment for Roadmap and Path plotting
    // LOG("Computation Time: " << result.compTime);
    // printf("\n");
    // Visualizer::makeFigure(problem, result.path);
    // Visualizer::showFigures();

    return result.path;
}

void MyDecentralizedMultiAgentRRT::createTree(const amp::MultiAgentProblem2D& problem, MyDecentralizedMultiAgentRRT::DecentralizedMultiAgentRRTResult& result, bool& resultFound) {
    bool isCollision{false};
    double samplex{0.0};
    double sampley{0.0};
    int numBranchDivisions{8};
    double fraction = 1.0 / numBranchDivisions;
    int agentIndex = result.path.agent_paths.size() - 1;
    resultFound = false;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disx(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disy(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> prob(0.0, 1.0);

    result.sampledPoints->push_back(problem.agent_properties[agentIndex].q_init);

    for (int i = 0; i < nSamples; i++) {
        double goalSample = prob(gen);
        if (goalSample <= goalProb) {
            samplex = problem.agent_properties[agentIndex].q_goal[0];
            sampley = problem.agent_properties[agentIndex].q_goal[1];
        } else {
            samplex = disx(gen);
            sampley = disy(gen);
        }

        double closestNodeNorm{std::numeric_limits<double>::max()};
        double checkNodeNorm{0.0};
        Eigen::Vector2d q_near{};
        Eigen::Vector2d q_check{};
        Eigen::Vector2d q_rand{samplex, sampley};
        for (int j = 0; j < result.sampledPoints->size(); j++) {
            q_check = result.sampledPoints->at(j);
            if ((checkNodeNorm = (q_rand - q_check).norm()) < closestNodeNorm) {
                q_near = q_check;
                closestNodeNorm = checkNodeNorm;
            }
        }

        Eigen::Vector2d q_new = q_near + (stepSize * (q_rand - q_near).normalized());
        Eigen::Vector2d direction = q_near - q_new;
        Eigen::Vector2d q_branch{};

        for (int m = 0; m < numBranchDivisions; m++) {
            q_branch = q_new + (direction * (m * fraction));
            isCollision = checkObstacleCollisions(problem, q_near, q_branch, agentIndex);
            if (isCollision) {
                continue;
            } else {
                int timeStep{1};
                uint32_t parentNode = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), q_near) - result.sampledPoints->begin());

                while (1) {
                    if (parentNode == 0) {
                        break;
                    } else {
                        parentNode = result.roadmap->parents(parentNode)[0];
                    }

                    timeStep++;
                }

                for (int j = 0; j < agentIndex; j++) {
                    isCollision = checkRobotCollision(problem, result, q_near, q_branch, agentIndex, j, timeStep);
                    if (isCollision) {
                        break;
                    }
                }

                if (isCollision) {
                    continue;
                }
            }

            if (!isCollision) {
                result.sampledPoints->push_back(q_branch);
                uint32_t nearIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), q_near) - result.sampledPoints->begin());
                uint32_t newIndex = static_cast<Node>(result.sampledPoints->size() - 1);
                result.roadmap->connect(nearIndex, newIndex, stepSize);

                double goalDist{};
                if ((goalDist = distanceL2(q_branch, problem.agent_properties[agentIndex].q_goal)) <= epsilon) {
                    resultFound = true;
                    result.sampledPoints->push_back(problem.agent_properties[agentIndex].q_goal);
                    uint32_t goalIndex = static_cast<Node>(result.sampledPoints->size() - 1);
                    result.roadmap->connect(newIndex, goalIndex, goalDist);
                    return;
                } else {
                    break;
                }
            }
        }
    }

    return;
}

bool MyDecentralizedMultiAgentRRT::checkObstacleCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& q_near, const Eigen::Vector2d& q_new, const int& agentNum) {
    int numSubdivisions{6};
    Eigen::Vector2d direction = q_new - q_near;

    for (auto obstacle : problem.obstacles) {
        Eigen::Vector2d c{0.0, 0.0};
        double distToObs = d_iq(q_new, obstacle, c);

        if (distToObs <= problem.agent_properties[agentNum].radius) {
            return true;
        } else if (distToObs <= distanceL2(q_new, q_near) + problem.agent_properties[agentNum].radius) {
            for (int m = 0; m < numSubdivisions; m++) {
                int numPoints = pow(2, m);
                double fraction = 1.0 / (2 * numPoints);
                for (int n = 0; n < numPoints; n++) {
                    Eigen::Vector2d subPoint = q_near + (direction * (fraction + (2 * n * fraction)));
                    distToObs = d_iq(subPoint, obstacle, c);
                    if (distToObs <= problem.agent_properties[agentNum].radius) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

bool MyDecentralizedMultiAgentRRT::checkRobotCollision(const amp::MultiAgentProblem2D& problem, MyDecentralizedMultiAgentRRT::DecentralizedMultiAgentRRTResult& result, const Eigen::Vector2d& q_near, const Eigen::Vector2d& q_new, const int& agentNum, const int& agentNumHighPri, const int& timeStep) {
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

bool MyDecentralizedMultiAgentRRT::highPriGoalCollision(const amp::MultiAgentProblem2D& problem, MyDecentralizedMultiAgentRRT::DecentralizedMultiAgentRRTResult& result, const int& agentNum, int& agentCollisionNum) {
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
