#include "CollisionDetector.h"

bool collisionPointPolygon(const Eigen::Vector2d& point, const amp::Polygon& poly) {
    bool collisionValue = true;
    std::vector<Eigen::Vector2d> vertices = poly.verticesCCW();
    int numVertices = vertices.size();

    for (int i = 0; i < numVertices; i++) {
        Eigen::Vector2d edge = (vertices[(i + 1) % numVertices] - vertices[i]).normalized();
        Eigen::Vector2d normal = Eigen::Vector2d(-edge[1], edge[0]).normalized();
        Eigen::Vector2d position = (point - vertices[i]).normalized();

        if (normal.dot(position) < 0) {
            collisionValue = false;
            break;
        }
    }

    return collisionValue;
}

bool collisionLineLine(const std::vector<Eigen::Vector2d>& line1, const std::vector<Eigen::Vector2d>& line2) {
    bool collisionValue = false;
    
    int orientation1 = pointOrientation(line1[0], line1[1], line2[0]);
    int orientation2 = pointOrientation(line1[0], line1[1], line2[1]);
    int orientation3 = pointOrientation(line2[0], line2[1], line1[0]);
    int orientation4 = pointOrientation(line2[0], line2[1], line1[1]);

	if (orientation1 != orientation2 && orientation3 != orientation4) {
        collisionValue = true;
    } else if (orientation1 == 0 && onLineSegment(line1[0], line2[0], line1[1])) {
        collisionValue = true;
    } else if (orientation2 == 0 && onLineSegment(line1[0], line2[1], line1[1])) {
        collisionValue =  true;
    } else if (orientation3 == 0 && onLineSegment(line2[0], line1[0], line2[1])) {
        collisionValue = true;
    } else if (orientation4 == 0 && onLineSegment(line2[0], line1[1], line2[1])) {
        collisionValue = true;
    }

    return collisionValue;
}

// Given three collinear points p1, p2, and p3, 
// check if point q2 lies on line segment 'p1-p3' 
bool onLineSegment(const Eigen::Vector2d& p1, 
                   const Eigen::Vector2d& p2, 
                   const Eigen::Vector2d& p3) { 
	if (p2[0] <= std::max(p1[0], p3[0]) && p2[0] >= std::min(p1[0], p3[0]) && 
		p2[1] <= std::max(p1[1], p3[1]) && p2[1] >= std::min(p1[1], p3[1])) {
        return true;
    }

	return false; 
}

// To find orientation of ordered triplet (p1, p2, p3). 
// The function returns following values 
// 0 --> p1, p2 and p3 are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int pointOrientation(const Eigen::Vector2d& p1, 
                     const Eigen::Vector2d& p2, 
                     const Eigen::Vector2d& p3) {
	double val = (p2[1] - p1[1]) * (p3[0] - p2[0]) - 
			     (p2[0] - p1[0]) * (p3[1] - p2[1]); 

	if (val == 0.0) {
        return 0;
    }

	return (val > 0) ? 1 : 2;
}

bool collisionLinePolygon(const std::vector<Eigen::Vector2d>& line, const amp::Polygon& poly) {
    bool collisionValue = false;
    const std::vector<Eigen::Vector2d> vertices = poly.verticesCCW();
    int numVertices = vertices.size();
    std::vector<Eigen::Vector2d> polyLine{};

    for (int i = 0; i < numVertices; i++) {
        polyLine.push_back(vertices[i]);
        polyLine.push_back(vertices[(i + 1) % numVertices]);
        collisionValue = collisionLineLine(line, polyLine);
        if (collisionValue) {
            break;
        } else {
            polyLine.clear();
        }
    }

    return collisionValue;
}

bool collisionPolygonPolygon(const amp::Polygon& poly1, const amp::Polygon& poly2) {
    bool collisionValue = false;
    const std::vector<Eigen::Vector2d> vertices = poly1.verticesCCW();
    int numVertices = vertices.size();
    std::vector<Eigen::Vector2d> polyLine{};

    for (int i = 0; i < numVertices; i++) {
        polyLine.push_back(vertices[i]);
        polyLine.push_back(vertices[(i + 1) % numVertices]);
        collisionValue = collisionLinePolygon(polyLine, poly2);
        if (collisionValue) {
            break;
        } else {
            polyLine.clear();
        }
    }

    return collisionValue;
}

double d_iq(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, Eigen::Vector2d& c) {
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
std::vector<Eigen::Vector2d> findClosestCs(const Eigen::Vector2d& q, const amp::Obstacle2D& obst) {
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

double distanceL2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    Eigen::Vector2d diff = p1 - p2;
    return diff.norm();
}
