# include "MyCentralizedMultiAgentRRT.h"

using namespace amp;


amp::MultiAgentPath2D MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    // LOG("Plan Num: " << planNum);
    // planNum++;

    auto start = std::chrono::high_resolution_clock::now();
    CentralizedMultiAgentRRTResult result{};
    bool resultFound{false};

    for (int i = 0; i < problem.numAgents(); i++) {
        result.path.agent_paths.push_back(Path2D());
    }

    createTree(problem, result, resultFound);

    if (resultFound) {
        resultsFound++;
        Eigen::VectorXd config{};
        Node parentNode = result.sampledPoints->size() - 1;
        while (1) {
            config = result(parentNode);
            for (int i = 0; i < problem.numAgents(); i++) {
                result.path.agent_paths[i].waypoints.insert(result.path.agent_paths[i].waypoints.begin(), Eigen::Vector2d(config[2 * i], config[(2 * i) + 1]));
            }

            if (parentNode == 0) {
                break;
            } else {
                parentNode = result.roadmap->parents(parentNode)[0];
            }
        }
    } else {
        for (int i = 0; i < problem.numAgents(); i++) {
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
        }
    }

    result.treeSize = result.roadmap->nodes().size();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    result.compTime = static_cast<double>(duration.count()) * 1000;
    compTimeDataSet.push_back(result.compTime);
    treeSizeDataSet.push_back(result.treeSize);

    // std::vector<std::vector<Eigen::Vector2d>> collisions{};
    // if (HW8::check(result.path, problem, collisions, false)) {
    //     numValidSolutions++;
    // } else {
    //     if (resultFound && collisions.size() > 0) {
    //         LOG("Path Collision!");
    //         Visualizer::makeFigure(problem, result.path, collisions);
    //         Visualizer::showFigures();
    //     }
    // }

    // Uncomment for Roadmap and Path plotting
    // LOG("Tree Size: " << result.treeSize);
    // LOG("Computation Time: " << result.compTime);
    // result.roadmap->print();
    // printf("\n");
    // Visualizer::makeFigure(problem, result.path);
    // Visualizer::showFigures();

    return result.path;
}

void MyCentralizedMultiAgentRRT::createTree(const amp::MultiAgentProblem2D& problem, MyCentralizedMultiAgentRRT::CentralizedMultiAgentRRTResult& result, bool& resultFound) {
    result.sampledPoints = std::make_unique<std::vector<Eigen::VectorXd>>();
    result.roadmap = std::make_shared<amp::Graph<double>>();
    bool isCollision{false};
    std::vector<double> configs{};
    int numBranchDivisions{8};
    double fraction = 1.0 / numBranchDivisions;
    resultFound = false;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disx(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disy(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> prob(0.0, 1.0);

    for (auto agent : problem.agent_properties) {
        configs.push_back(agent.q_init[0]);
        configs.push_back(agent.q_init[1]);
    }
    Eigen::VectorXd qInit = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(configs.data(), configs.size());

    result.sampledPoints->push_back(qInit);

    for (int i = 0; i < nSamples; i++) {
        double goalSample = prob(gen);
        if (goalSample <= goalProb) {
            for (auto agent : problem.agent_properties) {
                configs.push_back(agent.q_goal[0]);
                configs.push_back(agent.q_goal[1]);
            }
        } else {
            for (auto agent : problem.agent_properties) {
                configs.push_back(disx(gen));
                configs.push_back(disy(gen));
            }
        }

        double closestNodeNorm{std::numeric_limits<double>::max()};
        double checkNodeNorm{0.0};
        Eigen::VectorXd q_near{};
        Eigen::VectorXd q_check{};
        Eigen::VectorXd q_rand = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(configs.data(), configs.size());
        for (int j = 0; j < result.sampledPoints->size(); j++) {
            q_check = result.sampledPoints->at(j);
            if ((checkNodeNorm = (q_rand - q_check).norm()) < closestNodeNorm) {
                q_near = q_check;
                closestNodeNorm = checkNodeNorm;
            }
        }

        Eigen::VectorXd q_new(2 * problem.numAgents());
        for (int j = 0; j < q_new.size(); j += 2) {
            Eigen::Vector2d q_near2d{q_near[j], q_near[j + 1]};
            Eigen::Vector2d q_rand2d{q_rand[j], q_rand[j + 1]};
            Eigen::Vector2d q_new2d{};

            if (distanceL2(q_near2d, q_rand2d) <= stepSize) {
                q_new2d = q_rand2d;
            } else {
                q_new2d = q_near2d + (stepSize * (q_rand2d - q_near2d).normalized());
            }

            q_new[j] = q_new2d[0];
            q_new[j + 1] = q_new2d[1];
        }
        Eigen::VectorXd direction = q_near - q_new;
        Eigen::VectorXd q_branch{};

        for (int m = 0; m < numBranchDivisions; m++) {
            q_branch = q_new + (direction * (m * fraction));
            for (int j = 0; j < problem.numAgents(); j++) {
                isCollision = checkObstacleCollisions(problem, q_near, q_branch, j);
                if (isCollision) {
                    break;
                } else {
                    isCollision = checkRobotCollisions(problem, q_near, q_branch, j);
                    if (isCollision) {
                        break;
                    }
                }
            }

            if (!isCollision) {
                double graphDist{0.0};
                for (int j = 0; j < q_branch.size(); j += 2) {
                    graphDist += distanceL2(Eigen::Vector2d(q_near[j], q_near[j + 1]), Eigen::Vector2d(q_branch[j], q_branch[j + 1]));
                }
                result.sampledPoints->push_back(q_branch);
                uint32_t nearIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), q_near) - result.sampledPoints->begin());
                uint32_t newIndex = static_cast<Node>(result.sampledPoints->size() - 1);
                result.roadmap->connect(nearIndex, newIndex, graphDist);

                double goalDist{0.0};
                bool atGoal{true};
                for (int j = 0; j < problem.numAgents(); j++) {
                    double agentToGoal = distanceL2(Eigen::Vector2d(q_branch[2 * j], q_branch[(2 * j) + 1]), problem.agent_properties[j].q_goal);
                    if (agentToGoal > epsilon) {
                        atGoal = false;
                        break;
                    }
                    goalDist += agentToGoal;
                }

                if (atGoal) {
                    resultFound = true;
                    configs.clear();
                    for (auto agent : problem.agent_properties) {
                        configs.push_back(agent.q_goal[0]);
                        configs.push_back(agent.q_goal[1]);
                    }
                    Eigen::VectorXd qGoal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(configs.data(), configs.size());
                    result.sampledPoints->push_back(qGoal);
                    uint32_t goalIndex = static_cast<Node>(result.sampledPoints->size() - 1);
                    result.roadmap->connect(newIndex, goalIndex, goalDist);
                    return;
                } else {
                    break;
                }
            }
        }

        configs.clear();
    }

    return;
}

bool MyCentralizedMultiAgentRRT::checkObstacleCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_new, const int& agentNum) {
    int numSubdivisions{6};
    Eigen::Vector2d agentCenter{q_new[2 * agentNum], q_new[(2 * agentNum) + 1]};
    Eigen::Vector2d nearCenter{q_near[2 * agentNum], q_near[(2 * agentNum) + 1]};
    Eigen::Vector2d direction = agentCenter - nearCenter;

    for (auto obstacle : problem.obstacles) {
        Eigen::Vector2d c{0.0, 0.0};
        double distToObs = d_iq(agentCenter, obstacle, c);

        if (distToObs <= problem.agent_properties[agentNum].radius) {
            return true;
        } else if (distToObs <= distanceL2(agentCenter, nearCenter) + problem.agent_properties[agentNum].radius) {
            for (int m = 0; m < numSubdivisions; m++) {
                int numPoints = pow(2, m);
                double fraction = 1.0 / (2 * numPoints);
                for (int n = 0; n < numPoints; n++) {
                    Eigen::Vector2d subPoint = nearCenter + (direction * (fraction + (2 * n * fraction)));
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

bool MyCentralizedMultiAgentRRT::checkRobotCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_new, const int& agentNum) {
    int numSubdivisions{6};
    Eigen::Vector2d agentCenter{q_new[2 * agentNum], q_new[(2 * agentNum) + 1]};
    Eigen::Vector2d nearCenter{q_near[2 * agentNum], q_near[(2 * agentNum) + 1]};
    Eigen::Vector2d direction = agentCenter - nearCenter;
    
    for (int k = agentNum + 1; k < problem.numAgents(); k++) {
        Eigen::Vector2d agentkCenter{q_new[2 * k], q_new[(2 * k) + 1]};
        Eigen::Vector2d nearkCenter{q_near[2 * k], q_near[(2 * k) + 1]};
        Eigen::Vector2d directionk = agentkCenter - nearkCenter;

        double sumRadii = problem.agent_properties[agentNum].radius + problem.agent_properties[k].radius;
        double agentCenterDist = distanceL2(agentCenter, agentkCenter);
        if (agentCenterDist <= 1.1 * sumRadii) {
            return true;
        }

        agentCenterDist = distanceL2(nearCenter, agentkCenter);
        if (agentCenterDist <= 1.1 * sumRadii) {
            return true;
        }

        agentCenterDist = distanceL2(nearkCenter, agentCenter);
        if (agentCenterDist <= 1.1 * sumRadii) {
            return true;
        }
        
        if ((distanceL2(nearCenter, agentkCenter) <= distanceL2(agentCenter, nearCenter) + sumRadii) || (distanceL2(nearkCenter, agentCenter) <= distanceL2(agentkCenter, nearkCenter) + sumRadii)) {
            for (int m = 0; m < numSubdivisions; m++) {
                int numPoints = pow(2, m);
                double fraction = 1.0 / (2 * numPoints);
                for (int n = 0; n < numPoints; n++) {
                    Eigen::Vector2d subPoint = nearCenter + (direction * (fraction + (2 * n * fraction)));
                    Eigen::Vector2d subkPoint = nearkCenter + (directionk * (fraction + (2 * n * fraction)));
                    agentCenterDist = distanceL2(subPoint, subkPoint);
                    if (agentCenterDist <= 1.1 * sumRadii) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}
