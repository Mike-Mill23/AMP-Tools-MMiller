# include "MyCentralizedMultiAgentRRT.h"

using namespace amp;


amp::MultiAgentPath2D MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    std::string timerKey{"CentralizedMultiAgentRRT_Timer"};
    Timer centralizedMultiAgentRRTTimer{timerKey};
    Profiler::getMostRecentProfile(timerKey);
    CentralizedMultiAgentRRTResult result{};

    for (int i = 0; i < problem.numAgents(); i++) {
        result.path.agent_paths.push_back(Path2D());
    }

    createTree(problem, result);

    ShortestPathProblem centralizedRRTProblem{};
    centralizedRRTProblem.graph = result.roadmap;
    centralizedRRTProblem.init_node = 0;
    centralizedRRTProblem.goal_node = result.sampledPoints->size() - 1;

    CentralizedRRTSearchHeuristic heuristic{centralizedRRTProblem, result.sampledPoints};

    MyAStar aStar{};
    MyAStar::GraphSearchResult nodePath = aStar.search(centralizedRRTProblem, heuristic);

    if (nodePath.success) {
        for (auto n : nodePath.node_path) {
            Eigen::VectorXd nodeConfigs = result(n);
            for (int i = 0; i < problem.numAgents(); i++) {
                result.path.agent_paths[i].waypoints.push_back(Eigen::Vector2d(nodeConfigs[2 * i], nodeConfigs[(2 * i) + 1]));
            }
        }
    } else {
        for (int i = 0; i < problem.numAgents(); i++) {
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_init);
            result.path.agent_paths[i].waypoints.push_back(problem.agent_properties[i].q_goal);
        }
    }

    result.treeSize = result.roadmap->nodes().size();
    result.compTime = Profiler::getMostRecentProfile(timerKey);
    compTimeDataSet.push_back(result.compTime);
    treeSizeDataSet.push_back(result.treeSize);

    // Uncomment for Roadmap and Path plotting
    LOG("Tree Size: " << result.treeSize);
    LOG("Computation Time: " << result.compTime);
    printf("\n");
    Visualizer::makeFigure(problem, result.path);
    Visualizer::showFigures();

    return result.path;
}

void MyCentralizedMultiAgentRRT::createTree(const amp::MultiAgentProblem2D& problem, MyCentralizedMultiAgentRRT::CentralizedMultiAgentRRTResult& result) {
    result.sampledPoints = std::make_unique<pointVec>();
    result.roadmap = std::make_shared<amp::Graph<double>>();
    bool isCollision{false};
    std::vector<double> configs{};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disx(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disy(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> prob(0.0, 1.0);

    point_t qInit{};
    for (auto agent : problem.agent_properties) {
        qInit.push_back(agent.q_init[0]);
        qInit.push_back(agent.q_init[1]);
    }

    result.sampledPoints->push_back(qInit);

    for (int i = 0; i < nSamples; i++) {
        KDTree mapTree{*(result.sampledPoints)};

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

        point_t nearestPoint = mapTree.nearest_point(configs);
        Eigen::VectorXd q_near = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(nearestPoint.data(), nearestPoint.size());
        Eigen::VectorXd q_rand = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(configs.data(), configs.size());
        Eigen::VectorXd q_new(2 * problem.numAgents());
        for (int j = 0; j < q_new.size(); j += 2) {
            Eigen::Vector2d q_near2d{q_near[j], q_near[j + 1]};
            Eigen::Vector2d q_rand2d{q_rand[j], q_rand[j + 1]};
            Eigen::Vector2d q_new2d = q_near2d + (stepSize * (q_rand2d - q_near2d).normalized());
            q_new[j] = q_new2d[0];
            q_new[j + 1] = q_new2d[1];
        }

        for (int j = 0; j < problem.numAgents(); j++) {
            isCollision = false;
            Eigen::Vector2d agentCenter{q_new[2 * j], q_new[(2 * j) + 1]};

            for (auto obstacle : problem.obstacles) {
                Eigen::Vector2d c{0.0, 0.0};
                double distToObs = d_iq(agentCenter, obstacle, c);

                if (distToObs <= problem.agent_properties[j].radius) {
                    isCollision = true;
                    break;
                }
            }

            if (isCollision) {
                break;
            } else {
                for (int k = j + 1; k < problem.numAgents(); k++) {
                    double agentCenterDist = distanceL2(agentCenter, Eigen::Vector2d(q_new[2 * k], q_new[(2 * k) + 1]));
                    double sumRadii = problem.agent_properties[j].radius + problem.agent_properties[k].radius;
                    if (agentCenterDist <= sumRadii) {
                        isCollision = true;
                        break;
                    }
                }
            }

            if (isCollision) {
                break;
            }
        }

        if (!isCollision) {
            result.sampledPoints->push_back(point_t{q_new.data(), q_new.data() + q_new.size()});
            uint32_t nearIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), nearestPoint) - result.sampledPoints->begin());
            uint32_t newIndex = static_cast<Node>(result.sampledPoints->size() - 1);
            result.roadmap->connect(nearIndex, newIndex, stepSize * problem.numAgents());

            double goalDist{0.0};
            bool atGoal{true};
            for (int j = 0; j < problem.numAgents(); j++) {
                double agentToGoal = distanceL2(Eigen::Vector2d(q_new[2 * j], q_new[(2 * j) + 1]), problem.agent_properties[j].q_goal);
                if (agentToGoal > epsilon) {
                    atGoal = false;
                    break;
                }
                goalDist += agentToGoal;
            }

            if (atGoal) {
                point_t qGoal{};
                for (auto agent : problem.agent_properties) {
                    qGoal.push_back(agent.q_goal[0]);
                    qGoal.push_back(agent.q_goal[1]);
                }
                result.sampledPoints->push_back(qGoal);
                uint32_t goalIndex = static_cast<Node>(result.sampledPoints->size() - 1);
                result.roadmap->connect(newIndex, goalIndex, goalDist);
                break;
            }
        }

        configs.clear();
    }

    return;
}
