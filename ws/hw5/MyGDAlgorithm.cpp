# include "MyGDAlgorithm.h"

using namespace amp;


amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path{};
    std::vector<Eigen::Vector2d> momentum{};
    std::vector<double> ema{};
    Eigen::Vector2d gradient{0.0, 0.0};
    Eigen::Vector2d q_next{0.0, 0.0};
    Eigen::Vector2d q_step{0.0, 0.0};
    Eigen::Vector2d momentum_step{0.0, 0.0};
    double ema_step{0.0};
    double uValuePrev{0.0};
    double uValue{0.0};
    double stepSize{0.05};
    const int stepsAhead{50};
    int gradPerturbCount{1};

    path.waypoints.push_back(problem.q_init);
    momentum.push_back(Eigen::Vector2d(0.0, 0.0));
    ema.push_back(0.0);
    setStarParams(problem);
    uValuePrev = U(path.waypoints.back(), problem);

    // while (gradU(path.waypoints.back(), problem).norm() >= epsilon || distanceL2(path.waypoints.back(), problem.q_goal) > d_star_goal) {
    for (int i = 0; i < 10000; i++) {
        gradient = gradU(path.waypoints.back(), problem);
        lookaheadForLocalMin(path.waypoints.back(), q_step, stepsAhead, gradient, momentum, momentum_step, ema, ema_step, problem);
        // double startAngle = atan2(-gradient.normalized()[1], -gradient.normalized()[0]);
        // gradPerturbCount = 1;
        // while (lookaheadForLocalMin(path.waypoints.back(), q_step, stepsAhead, gradient, momentum, momentum_step, problem)) {
        //     double perturbAngle = (2 * M_PI * gradPerturbCount / 180) + startAngle;

        //     gradient[0] = gradient[0] * cos(perturbAngle) - gradient[1] * sin(perturbAngle);
        //     gradient[1] = gradient[0] * sin(perturbAngle) + gradient[1] * cos(perturbAngle);

        //     gradPerturbCount++;
        //     if (gradPerturbCount >= 180) {
        //         break;
        //     }
        // }
        path.waypoints.push_back(q_step);
        momentum.push_back(momentum_step);
        ema.push_back(ema_step);
    }

    path.waypoints.push_back(problem.q_goal);
    return path;
}

bool MyGDAlgorithm::lookaheadForLocalMin(Eigen::Vector2d q_curr, Eigen::Vector2d& q_step, const int& stepsAhead, Eigen::Vector2d gradient0, 
                                         std::vector<Eigen::Vector2d> momentum, Eigen::Vector2d& momentum_step, 
                                         std::vector<double> ema, double& ema_step, const amp::Problem2D& problem) {
    double uValuePrev{0.0};
    double uValue{0.0};
    double stepSize{0.5};
    std::vector<Eigen::Vector2d> q_next{};
    Eigen::Vector2d q_search{0.0, 0.0};
    Eigen::Vector2d gradient{0.0, 0.0};
    int momentumSize0 = momentum.size();
    int emaSize0 = ema.size();

    uValuePrev = U(q_curr, problem);
    q_next.push_back(q_curr);

    for (int i = 0; i < stepsAhead; i++) {
        if (i == 0) {
            gradient = gradient0;
        } else {
            gradient = gradU(q_curr, problem);
        }
        momentum.push_back((beta1 * momentum.back()) + ((1 - beta1) * gradient));
        ema.push_back((beta2 * ema.back()) + ((1 - beta2) * gradient.dot(gradient)));
        q_next.push_back(q_next.back() - (momentum.back() / (1 - beta1)) * (alpha / (sqrt(ema.back() / (1 - beta2)) + 0.00000001)));
        // momentum.push_back(q_curr - (alpha * gradient));
        // q_next.push_back(momentum.back() + (beta * (momentum.back() - momentum.end()[-2])));

        uValue = U(q_next.back(), problem);

        if ((uValuePrev - uValue) <= 0.01) {
            q_next.pop_back();
            momentum.pop_back();
            ema.pop_back();

            double startAngle = atan2(-gradient.normalized()[1], -gradient.normalized()[0]);
            for (int j = 0; j < 180; j++) {
                double searchAngle = (2 * M_PI * j / 180) + startAngle;
                q_search[0] = q_curr[0] + (stepSize * cos(searchAngle));
                q_search[1] = q_curr[1] + (stepSize * sin(searchAngle));

                if ((U(q_search, problem) - uValuePrev) <= -0.1) {
                    //gradient = (q_search - q_curr).normalized() * gradU(q_search, problem).norm();
                    gradient = gradU(q_search, problem);

                    momentum.push_back((beta1 * momentum.back()) + ((1 - beta1) * gradient));
                    ema.push_back((beta2 * ema.back()) + ((1 - beta2) * gradient.dot(gradient)));
                    q_next.push_back(q_next.back() - (momentum.back() / (1 - beta1)) * (alpha / (sqrt(ema.back() / (1 - beta2)) + 0.00000001)));
                    // momentum.push_back(q_curr - (alpha * gradient));
                    // q_next.push_back(momentum.back() + (beta * (momentum.back() - momentum.end()[-2])));
                    break;
                }

                if (j == 179) {
                    if (distanceL2(q_curr, problem.q_goal) <= d_star_goal) {
                        if (i == 0) {
                            q_step = q_next.begin()[0];
                            momentum_step = momentum.begin()[momentumSize0 - 1];
                            ema_step = ema.begin()[emaSize0 - 1];
                        } else {
                            q_step = q_next.begin()[1];
                            momentum_step = momentum.begin()[momentumSize0];
                            ema_step = ema.begin()[emaSize0];
                        }
                        return false;
                    } else {
                        LOG("Local Minima in " << i << " steps!");
                        return true;
                    }
                }
            }
        }

        uValuePrev = U(q_next.back(), problem);
        q_curr = q_next.back();
    }

    q_step = q_next.begin()[1];
    momentum_step = momentum.begin()[momentumSize0];
    ema_step = ema.begin()[emaSize0];
    return false;
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
    // LOG("d_star_goal: " << d_star_goal);
    // for (int i = 0; i < Q_star_i.size(); i++) {
    //     LOG("Q_star[" << i << "]: " << Q_star_i[i]);
    // }
}

double MyGDAlgorithm::U(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    double height{0.0};

    height += Uatt(q, problem);

    for (int i = 0; i < problem.obstacles.size(); i++) {
        height += Urepi(q, problem.obstacles[i], i);
    }

    return height;
}

double MyGDAlgorithm::Uatt(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    double height{0.0};
    double dist_goal = distanceL2(q, problem.q_goal);

    if (dist_goal <= d_star_goal) {
        height = 0.5 * xi * pow(dist_goal, 2);
    } else {
        height = (d_star_goal * xi * dist_goal) - (0.5 * xi * pow(d_star_goal, 2));
    }

    return height;
}

double MyGDAlgorithm::Urepi(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, int index) {
    double height{0.0};
    Eigen::Vector2d c{0.0, 0.0};
    double dist_iq = d_iq(q, obst, c);

    if (index >= Q_star_i.size()) {
        index = 0;
    }

    if (dist_iq <= Q_star_i[index]) {
        height = 0.5 * eta * pow(((1 / dist_iq) - (1 / Q_star_i[index])), 2);
    }

    return height;
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


/*-------- Debugging LOGs --------*/

// LOG("uValuePrev: " << uValuePrev);
// LOG("uValue: " << uValue);
// LOG("(uValuePrev - uValue) <= 0.01: " << ((uValuePrev - uValue) <= 0.01));
// LOG("q_curr:\n" << q_curr);
// LOG("q_next:\n" << q_next.back());
// LOG("momentum:\n" << momentum.back());
// LOG("-gradient:\n" << (-gradient));

// LOG("uValuePrev: " << uValuePrev);
// LOG("uValueSearch: " << U(q_search, problem));
// LOG("q_curr:\n" << q_curr);
// LOG("q_search:\n" << q_search);
// LOG("-gradient:\n" << (-gradient));
