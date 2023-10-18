# include "MyPotentialFunction.h"

using namespace amp;


MyPotentialFunction::MyPotentialFunction(double xi, double eta, double epsilon, amp::Problem2D problem) : xi(xi), eta(eta), epsilon(epsilon), problem(problem) {
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
    // for (int i = 0; i < Q_star_i.size(); i++) {
    //     Q_star_i[i] = 0.5;
    // }
}

double MyPotentialFunction::U(const Eigen::Vector2d& q) const {
    double height{0.0};

    height += Uatt(q);

    for (int i = 0; i < problem.obstacles.size(); i++) {
        height += Urepi(q, problem.obstacles[i], i);
    }

    return height;
}

double MyPotentialFunction::Uatt(const Eigen::Vector2d& q) const {
    double height{0.0};
    double dist_goal = distanceL2(q, problem.q_goal);

    if (dist_goal <= d_star_goal) {
        height = 0.5 * xi * pow(dist_goal, 2);
    } else {
        height = (d_star_goal * xi * dist_goal) - (0.5 * xi * pow(d_star_goal, 2));
    }

    return height;
}

double MyPotentialFunction::Urepi(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, int index) const {
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

double MyPotentialFunction::d_iq(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, Eigen::Vector2d& c) const {
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
std::vector<Eigen::Vector2d> MyPotentialFunction::findClosestCs(const Eigen::Vector2d& q, const amp::Obstacle2D& obst) const {
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

double MyPotentialFunction::distanceL2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const {
    Eigen::Vector2d diff = p1 - p2;
    return diff.norm();
}
