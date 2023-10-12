# include "MyGDAlgorithm.h"

using namespace amp;


amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path{};
    Eigen::Vector2d gradU{0.0, 0.0};
    path.waypoints.push_back(problem.q_init);

    

    path.waypoints.push_back(problem.q_goal);
    return path;
}

Eigen::Vector2d MyGDAlgorithm::gradU(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    Eigen::Vector2d gradient{0.0, 0.0};

    return gradient;
}

Eigen::Vector2d MyGDAlgorithm::gradUatt(const Eigen::Vector2d& q, const amp::Problem2D& problem) {
    Eigen::Vector2d gradient{0.0, 0.0};
    double dist_goal = distanceL2(q, problem.q_goal);

    if (dist_goal <= d_star_goal)

    return gradient;
}

Eigen::Vector2d MyGDAlgorithm::gradUrepi(const Eigen::Vector2d& q, const amp::Obstacle2D& obst) {
    Eigen::Vector2d gradient{0.0, 0.0};

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
