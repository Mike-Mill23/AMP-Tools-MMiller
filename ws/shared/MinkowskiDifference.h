#include "AMPCore.h"

#define _USE_MATH_DEFINES
#include <math.h>

amp::Polygon MinkowskiDifference(const amp::Polygon& robot, const amp::Polygon& obstacle);

std::vector<amp::Polygon> MinkowskiDifference(const amp::Polygon& robot, const amp::Polygon& obstacle, 
                                 std::vector<double>& rotationAngles, const int numRotations);

double vertexAngle(const Eigen::Vector2d vi, const Eigen::Vector2d vip1);

amp::Polygon rotatePolygon(const amp::Polygon& poly, const double angle);
