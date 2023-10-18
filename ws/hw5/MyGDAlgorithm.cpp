# include "MyGDAlgorithm.h"

using namespace amp;


amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path{};
    Eigen::Vector2d gradient{0.0, 0.0};
    Eigen::Vector2d avgGradient{0.0, 0.0};
    Eigen::Vector2d stepGradient{0.0, 0.0};
    std::vector<Eigen::Vector2d> gradients{};
    bool waypointCollision{false};
    int count{0};

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> disReal(-1.0, 1.0);

    path.waypoints.push_back(problem.q_init);
    setStarParams(problem);

    while (gradU(path.waypoints.back(), problem).norm() >= epsilon || distanceL2(path.waypoints.back(), problem.q_goal) > d_star_goal) {
        avgGradient = Eigen::Vector2d(0.0, 0.0);
        gradient = gradU(path.waypoints.back(), problem);
        gradients.push_back(gradient);
        if (gradient.norm() < epsilon) {
            gradient += Eigen::Vector2d(disReal(gen), disReal(gen));
            path.waypoints.push_back(path.waypoints.back() - (0.1 * gradient));
            stepGradient = -gradU(path.waypoints.back(), problem);
            while (gradient.dot(stepGradient) <= 0 && (path.waypoints.back()[0] > problem.x_min && path.waypoints.back()[0] < problem.x_max 
                                                   && path.waypoints.back()[1] > problem.y_min && path.waypoints.back()[1] < problem.y_max)) {
                path.waypoints.push_back(path.waypoints.back() - (alpha * stepGradient));
                gradient = gradU(path.waypoints.back(), problem);

                if (((path.waypoints.back() - (alpha * gradient)) - path.waypoints.back()).norm() >= 1.0) {
                    path.waypoints.pop_back();
                    break;
                }

                for (int j = 0; j < problem.obstacles.size(); j++) {
                    std::vector<Eigen::Vector2d> waypointLine{path.waypoints.back(), path.waypoints.back() - (0.5 * stepGradient.normalized())};
                    amp::Obstacle2D obst = problem.obstacles[j];
                    if (waypointCollision = collisionLinePolygon(waypointLine, obst)) {
                        break;
                    }
                }
                
                if (waypointCollision) {
                    path.waypoints.pop_back();
                    break;
                }
            }
        } else {
            gradient += Eigen::Vector2d(disReal(gen), disReal(gen));
            path.waypoints.push_back(path.waypoints.back() - (alpha * gradient));
        }

        count++;
        if (count >= 100000) {
            break;
        }
    }

    path.waypoints.push_back(problem.q_goal);
    LOG("count: " << count);
    return path;
}

void MyGDAlgorithm::setStarParams(const amp::Problem2D& problem) {
    Eigen::Vector2d c{0.0, 0.0};
    d_star_goal = std::numeric_limits<double>::max();
    Q_star_i.clear();
    for (int i = 0; i < problem.obstacles.size(); i++) {
        double dist_goal_obst = d_iq(problem.q_goal, problem.obstacles[i], c);

        if (dist_goal_obst < d_star_goal) {
            d_star_goal = dist_goal_obst;
        }

        double perimeter{0.0};
        std::vector<Eigen::Vector2d> vertices = problem.obstacles[i].verticesCCW();
        int numVertices = vertices.size();
        for (int j = 0; j < numVertices; j++) {
            perimeter += distanceL2(vertices[j], vertices[(j + 1) % numVertices]);
        }
        Q_star_i.push_back(perimeter / (2 * numVertices));   // average side length / 2
    }
}

Eigen::Vector2d MyGDAlgorithm::gradU(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    Eigen::Vector2d gradient{0.0, 0.0};

    gradient += gradUatt(q, problem);

    for (int i = 0; i < problem.obstacles.size(); i++) {
        gradient += gradUrepi(q, problem.obstacles[i], i);
    }

    return gradient;
}

Eigen::Vector2d MyGDAlgorithm::gradUatt(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    Eigen::Vector2d gradient{0.0, 0.0};
    double dist_goal = distanceL2(q, problem.q_goal);

    if (dist_goal <= d_star_goal) {
        gradient = xi * (q - problem.q_goal);
    } else {
        gradient = (d_star_goal * xi * (q - problem.q_goal)) / dist_goal;
    }

    return gradient;
}

Eigen::Vector2d MyGDAlgorithm::gradUrepi(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, int index) {
    Eigen::Vector2d gradient{0.0, 0.0};
    Eigen::Vector2d c{0.0, 0.0};
    double dist_iq = d_iq(q, obst, c);

    if (index >= Q_star_i.size()) {
        index = 0;
    }

    if (dist_iq <= Q_star_i[index]) {
        double dist_qc = distanceL2(q, c);
        gradient = eta * ((1 / Q_star_i[index]) - (1 / dist_iq)) * ((q - c) / (dist_qc * pow(dist_iq, 2)));
    }

    return gradient;
}

double MyGDAlgorithm::d_iq(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, Eigen::Vector2d& c) {
    double dist_i{std::numeric_limits<double>::max()};
    std::vector<Eigen::Vector2d> listCs = findClosestCs(q, obst);

    for (int i = 0; i < listCs.size(); i++) {
        double dist_qc = distanceL2(q, listCs[i]);
        if (dist_qc < dist_i) {
            dist_i = dist_qc;
            c = listCs[i];
        }
    }

    return dist_i;
}

// Equation to find closest point on line segment to point off of segment from Stack Exchange:
// https://math.stackexchange.com/a/2193733
// f(t) = (1 − t)A + tB − P
// g(t) = t^2 ∥v∥^2 + 2t(v ⋅ u) + ∥u∥^2
std::vector<Eigen::Vector2d> MyGDAlgorithm::findClosestCs(const Eigen::Vector2d& q, const amp::Obstacle2D& obst) {
    std::vector<Eigen::Vector2d> vertices = obst.verticesCCW();
    int numVertices = vertices.size();
    std::vector<Eigen::Vector2d> sideCs{};

    for (int i = 0; i < numVertices; i++) {
        Eigen::Vector2d A = vertices[i];
        Eigen::Vector2d B = vertices[(i + 1) % numVertices];

        Eigen::Vector2d v = B - A;
        Eigen::Vector2d u = A - q;

        double t = -((v.dot(u)) / (v.dot(v)));

        if (t >= 0 && t <= 1) {
            Eigen::Vector2d sideC = ((1 - t) * A) + (t * B);
            sideCs.push_back(sideC);
        } else {
            double g0 = pow(u.norm(), 2);
            double g1 = pow(v.norm(), 2) + (2 * v.dot(u)) + pow(u.norm(), 2);
            
            if (g0 <= g1) {
                sideCs.push_back(A);
            } else {
                sideCs.push_back(B);
            }
        }
    }

    return sideCs;
}

double MyGDAlgorithm::distanceL2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    Eigen::Vector2d diff = p1 - p2;
    return diff.norm();
}
