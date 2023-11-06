# include "MyGoalBiasRRT.h"

using namespace amp;


amp::Path2D MyGoalBiasRRT::plan(const amp::Problem2D& problem) {
    std::string timerKey{"GoalBiasRRT_Timer"};
    Timer goalBiasRRTTimer{timerKey};
    GoalBiasRRTResult result{};

    createTree(problem, result);

    ShortestPathProblem rrtProblem{};
    rrtProblem.graph = result.roadmap;
    rrtProblem.init_node = 0;
    rrtProblem.goal_node = result.sampledPoints->size() - 1;

    RRTSearchHeuristic heuristic{rrtProblem, result.sampledPoints};

    MyAStar aStar{};
    MyAStar::GraphSearchResult nodePath = aStar.search(rrtProblem, heuristic);

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

    // Uncomment for Roadmap and Path plotting
    // LOG("Path Length: " << result.pathLength);
    // LOG("Computation Time: " << result.compTime);
    // printf("\n");
    // Visualizer::makeFigure<MyGoalBiasRRT::GoalBiasRRTResult>(problem, *(result.roadmap), result);
    // Visualizer::makeFigure(problem, result.path);
    // Visualizer::showFigures();

    return result.path;
}

void MyGoalBiasRRT::createTree(const amp::Problem2D& problem, MyGoalBiasRRT::GoalBiasRRTResult& result) {
    result.sampledPoints = std::make_unique<pointVec>();
    result.roadmap = std::make_shared<amp::Graph<double>>();
    bool isCollision{false};
    double samplex{0.0};
    double sampley{0.0};

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disx(problem.x_min, problem.x_max);
    std::uniform_real_distribution<> disy(problem.y_min, problem.y_max);
    std::uniform_real_distribution<> prob(0.0, 1.0);

    result.sampledPoints->insert(result.sampledPoints->begin(), point_t{problem.q_init[0], problem.q_init[1]});

    for (int i = 0; i < nSamples; i++) {
        KDTree mapTree{*(result.sampledPoints)};

        double goalSample = prob(gen);
        if (goalSample <= goalProb) {
            samplex = problem.q_goal[0];
            sampley = problem.q_goal[1];
        } else {
            samplex = disx(gen);
            sampley = disy(gen);
        }

        point_t nearestPoint = mapTree.nearest_point(point_t{samplex, sampley});
        Eigen::Vector2d q_near{nearestPoint[0], nearestPoint[1]};
        Eigen::Vector2d q_rand{samplex, sampley};
        Eigen::Vector2d q_new = q_near + (stepSize * (q_rand - q_near).normalized());

        isCollision = false;
        std::vector<Eigen::Vector2d> line{q_near, q_new};
        for (auto obstacle : problem.obstacles) {
            if (isCollision = collisionLinePolygon(line, obstacle)) {
                break;
            }
        }

        if (!isCollision) {
            result.sampledPoints->push_back(point_t{q_new[0], q_new[1]});
            uint32_t nearIndex = static_cast<Node>(std::find(result.sampledPoints->begin(), result.sampledPoints->end(), point_t{q_near[0], q_near[1]}) - result.sampledPoints->begin());
            uint32_t newIndex = static_cast<Node>(result.sampledPoints->size() - 1);
            result.roadmap->connect(nearIndex, newIndex, stepSize);

            double goalDist{};
            if ((goalDist = distanceL2(q_new, problem.q_goal)) <= epsilon) {
                result.sampledPoints->push_back(point_t{problem.q_goal[0], problem.q_goal[1]});
                uint32_t goalIndex = static_cast<Node>(result.sampledPoints->size() - 1);
                result.roadmap->connect(newIndex, goalIndex, goalDist);
                break;
            }
        }
    }

    return;
}

void MyGoalBiasRRT::smoothPath(const amp::Problem2D& problem, MyGoalBiasRRT::GoalBiasRRTResult& result) {
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
