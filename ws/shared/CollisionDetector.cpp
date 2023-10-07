#include "CollisionDetector.h"

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
