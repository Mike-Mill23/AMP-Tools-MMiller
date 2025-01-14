#include "AMPCore.h"

#define _USE_MATH_DEFINES
#include <math.h>

bool collisionPointPolygon(const Eigen::Vector2d& point, const amp::Polygon& poly);

// Line segment collision detection from GeeksForGeeks website:
// https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
bool collisionLineLine(const std::vector<Eigen::Vector2d>& line1, const std::vector<Eigen::Vector2d>& line2);

bool onLineSegment(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);

int pointOrientation(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);

bool collisionLinePolygon(const std::vector<Eigen::Vector2d>& line, const amp::Polygon& poly);

bool collisionPolygonPolygon(const amp::Polygon& poly1, const amp::Polygon& poly2);

double d_iq(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, Eigen::Vector2d& c);

std::vector<Eigen::Vector2d> findClosestCs(const Eigen::Vector2d& q, const amp::Obstacle2D& obst);

Eigen::Vector2d findClosestPoint(const Eigen::Vector2d& q, const std::vector<Eigen::Vector2d>& line);

double distanceL2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
