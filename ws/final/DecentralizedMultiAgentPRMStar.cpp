# include "DecentralizedMultiAgentPRMStar.h"

using namespace amp;


amp::MultiAgentPath2D DecentralizedMultiAgentPRMStar::plan(const amp::MultiAgentProblem2D& initProblem) {
    amp::MultiAgentProblem2D problem = initProblem;
    DecentralizedMultiAgentPRMStarResult result{static_cast<uint32_t>(initProblem.numAgents())};
    result.sampledPoints = std::make_unique<std::vector<Eigen::Vector2d>>();
    result.roadmap = std::make_shared<amp::Graph<double>>();

    sampleEnv(problem, result);
    createGraph(problem, result);

    for (int t = 0; t <= numTasks; t++) {
        // LOG("Task " << t << ":");
        if (t == numTasks) {
            for (int i = 0; i < problem.numAgents(); i++) {
                problem.agent_properties[i].q_goal = initProblem.agent_properties[i].q_init;
            }
        } else {
            assignTasks(problem, result);
        }

        for (int i = 0; i < problem.numAgents(); i++) {
            // LOG("\tAgent " << i << ":");
            // LOG("\t\tq_init: (" << problem.agent_properties[i].q_init[0] << ", " << problem.agent_properties[i].q_init[1] << ")");
            // LOG("\t\tq_goal: (" << problem.agent_properties[i].q_goal[0] << ", " << problem.agent_properties[i].q_goal[1] << ")");

            ShortestPathProblem prmStarProblem{};
            prmStarProblem.graph = result.roadmap;
            prmStarProblem.init_node = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), problem.agent_properties[i].q_init) - result.sampledPoints->begin());
            prmStarProblem.goal_node = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), problem.agent_properties[i].q_goal) - result.sampledPoints->begin());

            PRMStarSearchHeuristic heuristic{prmStarProblem, result.sampledPoints};

            MyAStar aStar{};
            MyAStar::GraphSearchResult nodePath = aStar.search(prmStarProblem, heuristic);

            if (nodePath.success) {
                for (auto n : nodePath.node_path) {
                    result.path.agent_paths[i].waypoints.push_back(result(n));
                }

                decentralizePath(problem, result, i);
            } else {
                result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
                result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
            }

            // problem.agent_properties[i].q_init = problem.agent_properties[i].q_goal;
        }

        MultiAgentPath2D taskPath = result.path;
        for (int i = 0; i < problem.numAgents(); i++) {
            auto initIt = std::find(taskPath.agent_paths[i].waypoints.begin(), taskPath.agent_paths[i].waypoints.end(), problem.agent_properties[i].q_init);
            taskPath.agent_paths[i].waypoints.erase(taskPath.agent_paths[i].waypoints.begin(), initIt);
        }
        
        Visualizer::makeFigure(problem, taskPath);

        for (int i = 0; i < problem.numAgents(); i++) {
            problem.agent_properties[i].q_init = problem.agent_properties[i].q_goal;
        }
    }

    //-------- Roadmap Visualization --------//
    // Problem2D roadmapProb{};
    // roadmapProb.x_min = problem.x_min;
    // roadmapProb.x_max = problem.x_max;
    // roadmapProb.y_min = problem.y_min;
    // roadmapProb.y_max = problem.y_max;
    // roadmapProb.obstacles = problem.obstacles;
    // roadmapProb.q_init = problem.agent_properties[0].q_init;
    // roadmapProb.q_goal = problem.agent_properties[0].q_goal;
    // Visualizer::makeFigure(roadmapProb, *(result.roadmap), result);

    return result.path;
}

void DecentralizedMultiAgentPRMStar::sampleEnv(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result) {
    bool isCollision{true};
    Eigen::Vector2d sample{};

    std::random_device rd;
    // unsigned int seed = rd();
    // printf("rd seed: %u\n", seed);
    // unsigned int seed = 745567386;
    // std::mt19937 gen(seed);
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
        result.sampledPoints->push_back(problem.agent_properties[i].q_init);
        // result.sampledPoints->insert(result.sampledPoints->begin() + (2 * i), problem.agent_properties[i].q_init);
        // result.sampledPoints->insert(result.sampledPoints->begin() + ((2 * i) + 1), problem.agent_properties[i].q_goal);
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

void DecentralizedMultiAgentPRMStar::assignTasks(amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> obs(0, problem.obstacles.size() - 1);
    bool isCollision{true};
    Eigen::Vector2d sample{};
    std::vector<Eigen::Vector2d> newGoals{};

    double max_radius = std::numeric_limits<double>::min();
    for (auto& agent : problem.agent_properties) {
        if (agent.radius > max_radius) {
            max_radius = agent.radius;
        }
    }

    for (int i = 0; i < problem.numAgents(); i++) {
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::min();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::min();

        Obstacle2D randObs = problem.obstacles[obs(gen)];
        std::vector<Eigen::Vector2d> vertices = randObs.verticesCCW();

        for (auto& vertex : vertices) {
            if (vertex[0] < x_min) {
                x_min = vertex[0];
            }

            if (vertex[0] > x_max) {
                x_max = vertex[0];
            }

            if (vertex[1] < y_min) {
                y_min = vertex[1];
            }

            if (vertex[1] > y_max) {
                y_max = vertex[1];
            }
        }

        std::uniform_real_distribution<> disx(x_min - (1.5 * max_radius), x_max + (1.5 * max_radius));
        std::uniform_real_distribution<> disy(y_min - (1.5 * max_radius), y_max + (1.5 * max_radius));
        isCollision = true;

        while (isCollision) {
            sample << disx(gen), disy(gen);
            isCollision = checkObstacleCollisions(problem, sample);
        }

        newGoals.push_back(sample);
        result.sampledPoints->push_back(sample);
    }

    std::vector<std::vector<double>> costMatrix{};
    for (int i = 0; i < problem.numAgents(); i++) {
        std::vector<double> costMatrixRow{};

        for (int j = 0; j < newGoals.size(); j++) {
            costMatrixRow.push_back(distanceL2(problem.agent_properties[i].q_init, newGoals[j]));
        }

        costMatrix.push_back(costMatrixRow);
    }

    HungarianAlgorithm hungAlgo;
    std::vector<int> assignments{};
    double cost = hungAlgo.Solve(costMatrix, assignments);

    for (int i = 0; i < problem.numAgents(); i++) {
        problem.agent_properties[i].q_goal = newGoals[assignments[i]];
    }

    pointVec kdPoints{};
    for (auto eigVec : *(result.sampledPoints)) {
        point_t kdPoint{eigVec[0], eigVec[1]};
        kdPoints.push_back(kdPoint);
    }

    isCollision = false;
    KDTree mapTree{kdPoints};
    double gammaPRM = (2 * pow(1 + (1/d), 1/d) * pow(mu_free/zeta_d, 1/d)) + 0.001;
    double prmStarRad = gammaPRM * pow(log(nSamples)/nSamples, 1/d);

    for (int i = 0; i < newGoals.size(); i++) {
        Eigen::Vector2d currentSampleEig{newGoals[i]};
        point_t currentSampleVec{currentSampleEig[0], currentSampleEig[1]};
        uint32_t nodeIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), currentSampleEig) - result.sampledPoints->begin());
        pointVec neighbors = mapTree.neighborhood_points(currentSampleVec, prmStarRad);

        for (auto neighbor : neighbors) {
            Eigen::Vector2d neighborEig{neighbor[0], neighbor[1]};
            isCollision = checkEdgeCollisions(problem, currentSampleEig, neighborEig);

            if (!isCollision) {
                double nodeDist = distanceL2(currentSampleEig, neighborEig);
                uint32_t neighborNodeIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), neighborEig) - result.sampledPoints->begin());
                result.roadmap->connect(nodeIndex, neighborNodeIndex, nodeDist);
                result.roadmap->connect(neighborNodeIndex, nodeIndex, nodeDist);
            }
        }
    }

    return;
}

void DecentralizedMultiAgentPRMStar::decentralizePath(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const int& agentNum) {
    std::vector<Eigen::Vector2d> agentPath = result.path.agent_paths[agentNum].waypoints;
    int numBackups = 0;

    recursiveBackup(problem, result, agentNum, agentPath, numBackups);

    return;
}

void DecentralizedMultiAgentPRMStar::recursiveBackup(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const int& agentNum, const std::vector<Eigen::Vector2d>& path, int numBackups) {
    size_t pathSize = path.size();
    bool isCollision{false};

    for (int hiPriAgentNum = 0; hiPriAgentNum < agentNum; hiPriAgentNum++) {
        for (int i = 0; i < pathSize - 2; i++) {
            isCollision = checkRobotCollision(problem, result, path[i], path[i + 1], agentNum, hiPriAgentNum, i);

            if (isCollision) {
                int stationaryIndex = std::find(result.path.agent_paths[agentNum].waypoints.begin(), result.path.agent_paths[agentNum].waypoints.end(), path[i]) - result.path.agent_paths[agentNum].waypoints.begin();
                std::vector<Eigen::Vector2d> modPath{};
                size_t modPathSize{};

                while (true) {
                    numBackups++;
                    modPath.clear();
                    modPath = result.path.agent_paths[agentNum].waypoints;
                    modPath.insert(modPath.begin() + stationaryIndex + 1, numBackups, modPath[stationaryIndex]);
                    modPathSize = modPath.size();

                    for (int j = stationaryIndex; j < modPathSize - 2; j++) {
                        isCollision = checkRobotCollision(problem, result, modPath[j], modPath[j + 1], agentNum, hiPriAgentNum, j);
                        
                        if (isCollision) {
                            if (stationaryIndex > 0) {
                                stationaryIndex--;
                            }
                            break;
                        }
                    }

                    if (numBackups > result.path.agent_paths[hiPriAgentNum].waypoints.size()) {
                        LOG("recursiveBackup(): Number of backups exceeded, cannot decentralize path.");
                        return;
                    } else if (!isCollision) {
                        recursiveBackup(problem, result, agentNum, modPath, numBackups);
                        return;
                    }
                }
            }
        }
    }

    result.path.agent_paths[agentNum].waypoints = path;
    return;
}

bool DecentralizedMultiAgentPRMStar::checkEdgeCollisions(const amp::MultiAgentProblem2D& problem, const Eigen::Vector2d& current, const Eigen::Vector2d& neighbor) {
    int numSubdivisions{8};
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
        if (collisionPointPolygon(config, obstacle)) {
            return true;
        }

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

bool DecentralizedMultiAgentPRMStar::checkRobotCollision(const amp::MultiAgentProblem2D& problem, DecentralizedMultiAgentPRMStar::DecentralizedMultiAgentPRMStarResult& result, const Eigen::Vector2d& curr, const Eigen::Vector2d& next, const int& agentNum, const int& hiPriAgentNum, const int& timeStep) {
    Eigen::Vector2d next_other{};
    Eigen::Vector2d curr_other{};
    
    if (timeStep > result.path.agent_paths[hiPriAgentNum].waypoints.size() - 2) {
        next_other = result.path.agent_paths[hiPriAgentNum].waypoints[result.path.agent_paths[hiPriAgentNum].waypoints.size() - 1];
        curr_other = next_other;
    } else {
        next_other = result.path.agent_paths[hiPriAgentNum].waypoints[timeStep + 1];
        curr_other = result.path.agent_paths[hiPriAgentNum].waypoints[timeStep];
    }

    std::vector<Eigen::Vector2d> pathLine{curr, next};
    std::vector<Eigen::Vector2d> pathLine_other{curr_other, next_other};
    if (collisionLineLine(pathLine, pathLine_other)) {
        return true;
    }

    double sumRadii = problem.agent_properties[agentNum].radius + problem.agent_properties[hiPriAgentNum].radius;
    Eigen::Vector2d c = findClosestPoint(curr, pathLine_other);
    double agentCenterDist = distanceL2(curr, c);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    c = findClosestPoint(next, pathLine_other);
    agentCenterDist = distanceL2(next, c);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    c = findClosestPoint(curr_other, pathLine);
    agentCenterDist = distanceL2(curr_other, c);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    c = findClosestPoint(next_other, pathLine);
    agentCenterDist = distanceL2(next_other, c);
    if (agentCenterDist <= 1.1 * sumRadii) {
        return true;
    }

    return false;
}
