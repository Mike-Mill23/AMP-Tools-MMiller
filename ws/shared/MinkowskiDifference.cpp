#include "MinkowskiDifference.h"

amp::Polygon MinkowskiDifference(const amp::Polygon& robot, const amp::Polygon& obstacle) {
    std::vector<Eigen::Vector2d> robotVertices = robot.verticesCCW();
    std::vector<Eigen::Vector2d> obstacleVertices = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> cSpaceVertices{};
    const int numRobotVertices = robotVertices.size();
    const int numObstacleVertices = obstacleVertices.size();

    // Invert the robot Polygon to get the Minkowski difference instead of sum
    int start_index = 0;
    std::vector<Eigen::Vector2d> invRobotVertices{};
    for (int i = 0; i < numRobotVertices; i++) {
        invRobotVertices.push_back(robotVertices[i] * -1.0);
        if (invRobotVertices[i][1] < invRobotVertices[start_index][1]) {
            start_index = i;
        }
    }

    if (start_index > 0) {
        invRobotVertices.insert(invRobotVertices.end(), invRobotVertices.begin(), invRobotVertices.begin() + start_index);
        invRobotVertices.erase(invRobotVertices.begin(), invRobotVertices.begin() + start_index);
    }

    int i = 0;
    int j = 0;
    double obstacleAngle;
    double robotAngle;
    while (i < numObstacleVertices || j < numRobotVertices) {
        cSpaceVertices.push_back(obstacleVertices[i % numObstacleVertices] + invRobotVertices[j % numRobotVertices]);

        obstacleAngle = vertexAngle(obstacleVertices[i % numObstacleVertices], obstacleVertices[(i + 1) % numObstacleVertices]);
        robotAngle = vertexAngle(invRobotVertices[j % numRobotVertices], invRobotVertices[(j + 1) % numRobotVertices]);
        if (i == numObstacleVertices) {
            obstacleAngle = 2 * M_PI;
        } else if (j == numRobotVertices) {
            robotAngle = 2 * M_PI;
        }

        if (obstacleAngle < robotAngle) {
            i++;
        } else if (obstacleAngle > robotAngle) {
            j++;
        } else {
            i++;
            j++;
        }
    }

    return amp::Polygon(cSpaceVertices);
}

std::vector<amp::Polygon> MinkowskiDifference(const amp::Polygon& robot, const amp::Polygon& obstacle, 
                                              std::vector<double>& rotationAngles, const int numRotations) {
    std::vector<amp::Polygon> cSpaceObstacle{};
    std::vector<Eigen::Vector2d> robotVertices = robot.verticesCCW();
    std::vector<Eigen::Vector2d> obstacleVertices = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> cSpaceVertices{};
    const int numRobotVertices = robotVertices.size();
    const int numObstacleVertices = obstacleVertices.size();
    std::vector<Eigen::Vector2d> invRobotVertices{};

    for (int r = 0; r < numRotations; r++) {
        rotationAngles.push_back(2 * M_PI * r / numRotations);
        amp::Polygon rotatedRobot = rotatePolygon(robot, rotationAngles.back());
        std::vector<Eigen::Vector2d> rotatedRobotVertices = rotatedRobot.verticesCCW();

        // Invert the robot Polygon to get the Minkowski difference instead of sum
        int start_index = 0;
        for (int i = 0; i < numRobotVertices; i++) {
            invRobotVertices.push_back(rotatedRobotVertices[i] * -1.0);
            if (invRobotVertices[i][1] < invRobotVertices[start_index][1]) {
                start_index = i;
            }
        }

        // Rearrange the inverted robot vertices to start with lowest y value
        if (start_index > 0) {
            invRobotVertices.insert(invRobotVertices.end(), invRobotVertices.begin(), invRobotVertices.begin() + start_index);
            invRobotVertices.erase(invRobotVertices.begin(), invRobotVertices.begin() + start_index);
        }

        int i = 0;
        int j = 0;
        double obstacleAngle = 0;
        double robotAngle = 0;
        while (i < numObstacleVertices || j < numRobotVertices) {
            cSpaceVertices.push_back(obstacleVertices[i % numObstacleVertices] + invRobotVertices[j % numRobotVertices]);

            obstacleAngle = vertexAngle(obstacleVertices[i % numObstacleVertices], obstacleVertices[(i + 1) % numObstacleVertices]);
            robotAngle = vertexAngle(invRobotVertices[j % numRobotVertices], invRobotVertices[(j + 1) % numRobotVertices]);
            if (i == numObstacleVertices) {
                obstacleAngle = 2 * M_PI;
            } else if (j == numRobotVertices) {
                robotAngle = 2 * M_PI;
            }

            if (obstacleAngle < robotAngle) {
                i++;
            } else if (obstacleAngle > robotAngle) {
                j++;
            } else {
                i++;
                j++;
            }
        }

        cSpaceObstacle.push_back(amp::Polygon(cSpaceVertices));
        cSpaceVertices.clear();
        invRobotVertices.clear();
    }

    return cSpaceObstacle;
}

double vertexAngle(const Eigen::Vector2d vi, const Eigen::Vector2d vip1) {
    Eigen::Vector2d edge = vip1 - vi;
    double unwrappedAngle = atan2(edge[1], edge[0]);
    return (unwrappedAngle >= 0 ? unwrappedAngle : (2 * M_PI + unwrappedAngle));
}

// Assumes rotation about the first vertex in the list
amp::Polygon rotatePolygon(const amp::Polygon& poly, const double angle) {
    Eigen::Matrix2d R{{cos(angle), -sin(angle)}, {sin(angle), cos(angle)}};
    const std::vector<Eigen::Vector2d> polyVertices = poly.verticesCCW();
    const Eigen::Vector2d refPoint = polyVertices.front();
    std::vector<Eigen::Vector2d> newPolyVertices{};

    for (int i = 0; i < polyVertices.size(); i++) {
        newPolyVertices.push_back((R * (polyVertices[i] - refPoint)) + refPoint);
    }

    return amp::Polygon(newPolyVertices);
}


// LOG("i: " << i);
// LOG("j: " << j);
// LOG("cSpaceVertex:\n" << cSpaceVertices.back());
// LOG("Obstacle Angle: " << obstacleAngle);
// LOG("Robot Angle: " << robotAngle);

// LOG("Inv Robot Vertices:");
// LOG("Vertex 0: (" << invRobotVertices[0][0] << ", " << invRobotVertices[0][1] << ")");
// LOG("Vertex 1: (" << invRobotVertices[1][0] << ", " << invRobotVertices[1][1] << ")");
// LOG("Vertex 2: (" << invRobotVertices[2][0] << ", " << invRobotVertices[2][1] << ")");
// LOG("start_index: " << start_index);
// LOG("num rob vert: " << numRobotVertices);

// LOG("Inv Robot Vertices:");
// for (int i = 0; i < invRobotVertices.size(); i++) {
//     LOG("Vertex " << i << ": (" << invRobotVertices[i][0] << ", " << invRobotVertices[i][1] << ")");
// }

// LOG("");
// LOG("i: " << i);
// LOG("j: " << j);
// LOG("cSpaceVertex:\n" << cSpaceVertices.back());
// LOG("Obstacle Angle: " << obstacleAngle);
// LOG("Robot Angle: " << robotAngle);
// LOG("robot pt j:\n" << invRobotVertices.at(j % numRobotVertices));
// LOG("robot pt j + 1:\n" << invRobotVertices.at((j + 1) % numRobotVertices));
// LOG("j % numRobotVertices: " << (j % numRobotVertices));
// LOG("j + 1 % numRobotVertices: " << ((j + 1) % numRobotVertices));
