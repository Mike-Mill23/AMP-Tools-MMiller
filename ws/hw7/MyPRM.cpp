# include "MyPRM.h"

using namespace amp;


amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    std::string timerKey{"PRM_Timer"};
    Timer prmTimer{timerKey};
    PRMResult result{};

    sampleEnv(problem, result);

    createGraph(problem, result);

    ShortestPathProblem prmProblem{};
    prmProblem.graph = result.roadmap;
    prmProblem.init_node = 0;
    prmProblem.goal_node = result.sampledPoints->size() - 1;

    PRMSearchHeuristic heuristic{prmProblem, result.sampledPoints};

    MyAStar aStar{};
    MyAStar::GraphSearchResult nodePath = aStar.search(prmProblem, heuristic);

    if (nodePath.success) {
        for (auto n : nodePath.node_path) {
            result.path.waypoints.push_back(result(n));
        }

        if (pathSmoothing) {
            smoothPath(problem, result);
        }
    } else {
        result.path.waypoints.push_back(problem.q_init);
        result.path.waypoints.push_back(problem.q_goal);
    }

    result.pathLength = result.path.length();
    result.compTime = Profiler::getMostRecentProfile(timerKey);

    if (nodePath.success) {
        numValidSolutions++;
        pathLengthDataSet.push_back(result.pathLength);
    } else {
        pathLengthDataSet.push_back(0.0);
    }
    compTimeDataSet.push_back(result.compTime);

    // LOG("Path Length: " << result.pathLength);
    // LOG("Computation Time: " << result.compTime);
    // printf("\n");
    // Visualizer::makeFigure<MyPRM::PRMResult>(problem, *(result.roadmap), result);
    // Visualizer::makeFigure(problem, result.path);
    // Visualizer::showFigures();

    return result.path;
}

void MyPRM::sampleEnv(const amp::Problem2D& problem, MyPRM::PRMResult& result) {
    result.sampledPoints = std::make_unique<pointVec>();
    bool isCollision{false};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disx(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disy(problem.y_min, problem.y_max);

    for (int i = 0; i < nSamples; i++) {
        double samplex = disx(gen);
        double sampley = disy(gen);
        isCollision = false;

        for (int j = 0; j < problem.obstacles.size(); j++){
            if (isCollision = collisionPointPolygon(Eigen::Vector2d(samplex, sampley), problem.obstacles[j])) {
                break;
            }
        }

        if (!isCollision) {
            result.sampledPoints->push_back(point_t{samplex, sampley});
        }
    }
    result.sampledPoints->insert(result.sampledPoints->begin(), point_t{problem.q_init[0], problem.q_init[1]});
    result.sampledPoints->push_back(point_t{problem.q_goal[0], problem.q_goal[1]});

    return;
}

void MyPRM::createGraph(const amp::Problem2D& problem, MyPRM::PRMResult& result) {
    bool isCollision{false};
    result.roadmap = std::make_shared<amp::Graph<double>>();
    KDTree mapTree{*(result.sampledPoints)};

    for (int i = 0; i < result.sampledPoints->size(); i++) {
        pointVec neighbors = mapTree.neighborhood_points(result.sampledPoints->at(i), neighborRadius);
        for (auto neighbor : neighbors) {
            isCollision = false;
            for (int k = 0; k < problem.obstacles.size(); k++) {
                if (isCollision = collisionLinePolygon(std::vector<Eigen::Vector2d>{Eigen::Vector2d(result.sampledPoints->at(i)[0], result.sampledPoints->at(i)[1]), 
                                                                                    Eigen::Vector2d(neighbor[0], neighbor[1])}, 
                                                                                    problem.obstacles[k])) {
                    break;
                }
            }

            if (!isCollision) {
                double nodeDist = distanceL2(Eigen::Vector2d(result.sampledPoints->at(i)[0], result.sampledPoints->at(i)[1]), Eigen::Vector2d(neighbor[0], neighbor[1]));
                uint32_t nodeIndex = static_cast<Node>(i);
                uint32_t neighborNodeIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), neighbor) - result.sampledPoints->begin());
                result.roadmap->connect(nodeIndex, neighborNodeIndex, nodeDist);
            }
        }
    }

    return;
}

void MyPRM::smoothPath(const amp::Problem2D& problem, MyPRM::PRMResult& result) {
    std::random_device rd;
    std::mt19937 gen(rd());
    bool isCollision{false};
    int numWaypoints = result.path.waypoints.size();

    for (int k = 0; k < numWaypoints; k++) {
        isCollision = false;
        std::uniform_int_distribution<> dis(0, (result.path.waypoints.size() - 1));
        int sample1 = dis(gen);
        int sample2 = dis(gen);

        int i = sample1 <= sample2 ? sample1 : sample2;
        int j = sample1 > sample2 ? sample1 : sample2;
        for (auto obstacle : problem.obstacles) {
            if (isCollision = collisionLinePolygon(std::vector<Eigen::Vector2d>{result.path.waypoints[i], result.path.waypoints[j]}, obstacle)) {
                break;
            }
        }

        if (!isCollision && j > (i + 1)) {
            result.path.waypoints.erase(result.path.waypoints.begin() + (i + 1), result.path.waypoints.begin() + j);
        }
    }

    return;
}
