#include "MyBugAlgorithm.h"

using namespace amp;

// Implement your methods in the `.cpp` file, for example:
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {
    bool success;
    const double stepSize = 0.1;
    amp::Path2D path;
    std::vector<Eigen::Vector2d> collisions{};
    Eigen::Vector2d hitPoint{};
    Eigen::Vector2d leavePoint{};
    std::array<amp::Path2D, 2> splitPath{};

    path.waypoints.push_back(problem.q_init);

    while (!HW2::check(path, problem, false)) {
    //for (int i = 0; i < 16; i++) {
        success = step(problem, path, collisions, stepSize);

        if (!success) {
            LOG("followObstacle");
            followObstacle(problem, path, collisions, stepSize, hitPoint, leavePoint, splitPath);
            // break;
            LOG("gotoLeavePoint");
            gotoLeavePoint(path, hitPoint, leavePoint, splitPath);
        }

        collisions.clear();
        splitPath.fill(amp::Path2D());
    //}
    }

    // path.waypoints.push_back(problem.q_goal);

    return path;
}

bool Bug1Algorithm::step(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d>& collisions, 
                         double stepSize) {
    Eigen::Vector2d currentPosition = path.waypoints.back();
    Eigen::Vector2d nextPosition{};
    Eigen::Vector2d currentToNext{};
    Eigen::Vector2d currentToCollision{};
    std::vector<Eigen::Vector2d> tmpCollisionList{};
    double dist;
    double direction;
    bool success = false;
    bool isCollision;

    //LOG("Current Position:\n" << currentPosition);

    const double rise = problem.q_goal[1] - currentPosition[1];
    const double run = problem.q_goal[0] - currentPosition[0];
    const double hyp = sqrt(pow(rise, 2) + pow(run, 2));

    if (dist = distanceToGoal(problem, currentPosition) <= stepSize) {
        nextPosition = problem.q_goal;
    } else {
        nextPosition[0] = currentPosition[0] + (stepSize / hyp) * run;
        nextPosition[1] = currentPosition[1] + (stepSize / hyp) * rise;
    }

    amp::Path2D checkPath = path;
    checkPath.waypoints.push_back(nextPosition);
    checkPath.waypoints.push_back(problem.q_goal);

    HW2::check(checkPath, problem, tmpCollisionList, false);
    isCollision = isTrueCollision(problem, tmpCollisionList, currentPosition, nextPosition, stepSize);

    if (isCollision) {
        collisions.push_back(tmpCollisionList.front());
        if (collisions.size() > 2) {
            collisions.erase(collisions.begin());
        }
    } else {
        success = true;
        path.waypoints.push_back(nextPosition);
    }

    return success;
}

bool Bug1Algorithm::step(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d>& collisions, 
                         double stepSize, const double rise, const double run, const double hyp) {
    Eigen::Vector2d currentPosition = path.waypoints.back();
    Eigen::Vector2d nextPosition{};
    Eigen::Vector2d currentToNext{};
    Eigen::Vector2d currentToCollision{};
    std::vector<Eigen::Vector2d> tmpCollisionList{};
    double dist;
    double direction;
    bool success = false;
    bool isCollision;

    //LOG("Current Position:\n" << currentPosition);

    if (dist = distanceToGoal(problem, currentPosition) <= stepSize) {
        nextPosition = problem.q_goal;
    } else {
        nextPosition[0] = currentPosition[0] + (stepSize / hyp) * run;
        nextPosition[1] = currentPosition[1] + (stepSize / hyp) * rise;
    }

    amp::Path2D checkPath = path;
    checkPath.waypoints.push_back(nextPosition);
    checkPath.waypoints.push_back(problem.q_goal);

    HW2::check(checkPath, problem, tmpCollisionList, false);
    isCollision = isTrueCollision(problem, tmpCollisionList, currentPosition, nextPosition, stepSize);

    if (isCollision) {
        collisions.push_back(tmpCollisionList.front());
        if (collisions.size() > 2) {
            collisions.erase(collisions.begin());
        }
    } else {
        success = true;
        path.waypoints.push_back(nextPosition);
    }

    return success;
}

void Bug1Algorithm::followObstacle(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d>& collisions, 
                                   double stepSize, Eigen::Vector2d& hitPoint, Eigen::Vector2d& leavePoint, 
                                   std::array<amp::Path2D, 2>& splitPath) {
    hitPoint = path.waypoints.back();
    leavePoint = path.waypoints.back();
    Eigen::Vector2d directionWall = getRelativeVectorNormalized(hitPoint, collisions.back());
    Eigen::Vector2d directionTravel = directionWall;
    std::vector<Eigen::Vector2d> searchCollisionList{};
    double searchAngle0;

    do {
        searchAngle0 = atan2(directionWall[1], directionWall[0]);
        
        while (!searchSurroundings(problem, path, directionTravel, path.waypoints.back(), searchCollisionList, stepSize, searchAngle0)) {
            // directionTravel = getRelativeVectorNormalized(path.waypoints.back(), searchCollisionList.back());
            // Eigen::Vector2d nextStep(path.waypoints.back()[0] + (stepSize / directionTravel.norm()) * directionTravel[0], 
            //                          path.waypoints.back()[1] + (stepSize / directionTravel.norm()) * directionTravel[1]);
            // path.waypoints.pop_back();
            // directionTravel = getRelativeVectorNormalized(path.waypoints.back(), nextStep);
            // LOG("-------- Stuck in Search Surroundings --------");
            path.waypoints.pop_back();
            double distanceToCollision = distance(path.waypoints.back(), searchCollisionList.back());
            directionTravel = getRelativeVectorNormalized(path.waypoints.back(), searchCollisionList.back());
            searchAngle0 = atan2(directionTravel[1], directionTravel[0]);
            // LOG("Current Position:\n" << path.waypoints.back());
            // LOG("Collision point:\n" << searchCollisionList.back());
            step(problem, path, searchCollisionList, (distanceToCollision / 2.0), directionTravel[1], directionTravel[0], directionTravel.norm());
            // LOG("post step Position:\n" << path.waypoints.back());
            // LOG("post step collision point:\n" << searchCollisionList.back());
        }

        while (!step(problem, path, searchCollisionList, stepSize, directionTravel[1], directionTravel[0], directionTravel.norm())) {
            // Eigen::Vector2d collisionDirection = getRelativeVectorNormalized(path.waypoints.back(), searchCollisionList.back());
            // directionTravel = getRelativeVectorNormalized(searchCollisionList.front(), searchCollisionList.back());
            double currentAngle = atan2(directionTravel[1], directionTravel[0]);
            Eigen::Vector2d tmpDirectionTravel(cos(currentAngle + (M_PI / 2)), sin(currentAngle + (M_PI / 2)));
            step(problem, path, searchCollisionList, stepSize, tmpDirectionTravel[1], tmpDirectionTravel[0], tmpDirectionTravel.norm());
            // directionTravel[0] = cos(currentAngle + (M_PI / 2));
            // directionTravel[1] = sin(currentAngle + (M_PI / 2));
            // Eigen::Vector2d currentToLastCollision = getRelativeVectorNormalized(searchCollisionList.back(), searchCollisionList.front());
            // if (directionTravel.dot(currentToLastCollision)) {
            //     directionTravel = currentToLastCollision;
            // } else {
            //     directionTravel = getRelativeVectorNormalized(searchCollisionList.front(), searchCollisionList.back());
            // }
        }

        if (distanceToGoal(problem, path.waypoints.back()) < distanceToGoal(problem, leavePoint)) {
            if (splitPath[1].waypoints.size() > 0) {
                splitPath[0].waypoints.insert(splitPath[0].waypoints.end(), splitPath[1].waypoints.begin(), splitPath[1].waypoints.end());
                splitPath[1].waypoints.clear();
            }
            leavePoint = path.waypoints.back();
            splitPath[0].waypoints.push_back(leavePoint);
        } else {
            splitPath[1].waypoints.push_back(path.waypoints.back());
        }

        directionWall = getRelativeVectorNormalized(path.waypoints.back(), searchCollisionList.back());
    } while (sqrt(pow(path.waypoints.back()[0] - hitPoint[0],2) + pow(path.waypoints.back()[1] - hitPoint[1],2)) >= (0.95 * stepSize));

    path.waypoints.push_back(hitPoint);

    return;
}

bool Bug1Algorithm::searchSurroundings(const amp::Problem2D& problem, amp::Path2D& path, Eigen::Vector2d& directionTravel, 
                                      Eigen::Vector2d& currentPosition, std::vector<Eigen::Vector2d>& searchCollisionList, 
                                      double stepSize, const double startAngle) {
    const int numSearchPoints = 180;
    const int pathSize = path.waypoints.size();
    amp::Path2D searchRay = path;
    double searchAngle;
    double xSearch;
    double ySearch;
    double dist;
    double direction;
    bool isCollision = false;
    bool prevIsCollision = false;
    bool startWithCollision = false;
    std::vector<Eigen::Vector2d> edgePoints{};
    std::vector<Eigen::Vector2d> tmpCollisionList{};

    for (int i = 0; i < numSearchPoints; i++) {
        searchAngle = (2 * M_PI * i / numSearchPoints) + startAngle;
        xSearch = currentPosition[0] + (stepSize * cos(searchAngle));
        ySearch = currentPosition[1] + (stepSize * sin(searchAngle));
        Eigen::Vector2d nextSearch(xSearch, ySearch);

        searchRay.waypoints.push_back(nextSearch);
        searchRay.waypoints.push_back(problem.q_goal);
        tmpCollisionList.clear();
        HW2::check(searchRay, problem, tmpCollisionList, false);
        isCollision = isTrueCollision(problem, tmpCollisionList, currentPosition, nextSearch, stepSize);

        if (i == 0) {
            startWithCollision = isCollision;
        }

        if (startWithCollision) {
            // if (currentPosition[1] >= 13.0) {
            //     LOG("-------- Start in collision --------");
            //     LOG("edgepoints size: " << edgePoints.size());
            //     LOG("isCollision: " << isCollision);
            //     LOG("prevIsCollision: " << prevIsCollision);
            // }
            if (isCollision && prevIsCollision) {
                if (edgePoints.size() > 0) {
                    edgePoints[edgePoints.size() - 1] = tmpCollisionList.front();
                }
            } else if (isCollision && !prevIsCollision) {
                edgePoints.push_back(tmpCollisionList.front());
                if (edgePoints.size() == 2) {
                    calculateDirectionTravel(currentPosition, directionTravel, searchCollisionList, edgePoints);
                    break;
                }
            }
        } else {
            if (edgePoints.size() == 2) {
                if (isCollision) {
                    edgePoints[edgePoints.size() - 1] = tmpCollisionList.front();
                } else {
                    calculateDirectionTravel(currentPosition, directionTravel, searchCollisionList, edgePoints);
                    isCollision = prevIsCollision;
                    break;
                }
            } else {
                if (isCollision) {
                    edgePoints.push_back(tmpCollisionList.front());
                }
            }
        }

        prevIsCollision = isCollision;
        searchRay.waypoints.erase(searchRay.waypoints.begin() + pathSize, searchRay.waypoints.end());
    }

    LOG("-------- Return search surroundings --------");
    LOG("Current position:\n" << currentPosition);
    LOG("Direction Travel:\n" << directionTravel);
    LOG("collision list front:\n" << searchCollisionList.front());
    LOG("collision list back:\n" << searchCollisionList.back());
    LOG("isCollision: " << isCollision);
    return isCollision;
}

void Bug1Algorithm::calculateDirectionTravel(Eigen::Vector2d& currentPosition, Eigen::Vector2d& directionTravel, 
                                             std::vector<Eigen::Vector2d>& searchCollisionList, 
                                             std::vector<Eigen::Vector2d>& edgePoints) {
    Eigen::Vector2d point1 = getRelativeVectorNormalized(currentPosition, edgePoints.front());
    Eigen::Vector2d point2 = getRelativeVectorNormalized(currentPosition, edgePoints.back());

    double dotProdPoint1 = directionTravel.dot(point1);
    double dotProdPoint2 = directionTravel.dot(point2);

    double checkLeftPt1 = (directionTravel[0] * point1[1]) - (directionTravel[1] * point1[0]);
    double checkLeftPt2 = (directionTravel[0] * point2[1]) - (directionTravel[1] * point2[0]);

    // LOG("-------- calc dir travel --------");
    // LOG("Current Position:\n" << currentPosition);
    // LOG("edgePoint front:\n" << edgePoints.front());
    // LOG("edgePoint back:\n" << edgePoints.back());
    // LOG("left pt 1:\n" << checkLeftPt1);
    // LOG("left pt 2:\n" << checkLeftPt2);

    if (dotProdPoint1 * dotProdPoint2 < 0) {
        if (dotProdPoint1 > 0) {
            directionTravel = getRelativeVectorNormalized(point2, point1);
            searchCollisionList.clear();
            searchCollisionList.push_back(edgePoints.back());
            searchCollisionList.push_back(edgePoints.front());
        } else {
            directionTravel = getRelativeVectorNormalized(point1, point2);
            searchCollisionList.clear();
            searchCollisionList.push_back(edgePoints.front());
            searchCollisionList.push_back(edgePoints.back());
        }
    } else if (dotProdPoint1 + dotProdPoint2 < 0) {
        if (checkLeftPt1 * checkLeftPt2 > 0) {
            if (dotProdPoint1 > dotProdPoint2) {
                directionTravel = getRelativeVectorNormalized(point2, point1);
                searchCollisionList.clear();
                searchCollisionList.push_back(edgePoints.back());
                searchCollisionList.push_back(edgePoints.front());
            } else {
                directionTravel = getRelativeVectorNormalized(point1, point2);
                searchCollisionList.clear();
                searchCollisionList.push_back(edgePoints.front());
                searchCollisionList.push_back(edgePoints.back());
            }
        } else {
            if (checkLeftPt1 >= 0) {
                directionTravel = getRelativeVectorNormalized(point2, point1);
                searchCollisionList.clear();
                searchCollisionList.push_back(edgePoints.back());
                searchCollisionList.push_back(edgePoints.front());
            } else {
                directionTravel = getRelativeVectorNormalized(point1, point2);
                searchCollisionList.clear();
                searchCollisionList.push_back(edgePoints.front());
                searchCollisionList.push_back(edgePoints.back());
            }
        }
    } else {
        if (checkLeftPt1 >= 0) {
            directionTravel = getRelativeVectorNormalized(point2, point1);
            searchCollisionList.clear();
            searchCollisionList.push_back(edgePoints.back());
            searchCollisionList.push_back(edgePoints.front());
        } else {
            directionTravel = getRelativeVectorNormalized(point1, point2);
            searchCollisionList.clear();
            searchCollisionList.push_back(edgePoints.front());
            searchCollisionList.push_back(edgePoints.back());
        }
    }

    return;
}

void Bug1Algorithm::gotoLeavePoint(amp::Path2D& path, Eigen::Vector2d& hitPoint, Eigen::Vector2d& leavePoint, 
                    std::array<amp::Path2D, 2>& splitPath) {
    double distanceLeft = splitPath[0].length();
    double distanceRight = splitPath[1].length();

    if (distanceRight < distanceLeft) {
        path.waypoints.insert(path.waypoints.end(), splitPath[1].waypoints.rbegin(), splitPath[1].waypoints.rend());
    } else {
        path.waypoints.insert(path.waypoints.end(), splitPath[0].waypoints.begin(), splitPath[0].waypoints.end());
    }

    path.waypoints.push_back(leavePoint);

    return;
}

bool Bug1Algorithm::isTrueCollision(const amp::Problem2D& problem, std::vector<Eigen::Vector2d>& collisionList, 
                                  Eigen::Vector2d& currentPosition, Eigen::Vector2d& nextPosition, double stepSize) {
    bool isTrueCollision;
    double dist;
    double direction;
    Eigen::Vector2d currentToNext{};
    Eigen::Vector2d currentToCollision{};

    if (collisionList.size() > 0) {
        currentToNext = getRelativeVectorNormalized(currentPosition, nextPosition);
        currentToCollision = getRelativeVectorNormalized(currentPosition, collisionList.front());
        dist = distance(currentPosition, collisionList.front());
        direction = currentToNext.dot(currentToCollision);
    } else {
        dist = distanceToGoal(problem, currentPosition);
        direction = 0.0;
    }

    if (dist <= (1.01 * stepSize) && direction >= DIRECTION_EPSILON) {
        isTrueCollision = true;
    } else {
        isTrueCollision = false;
    }

    return isTrueCollision;
}

Eigen::Vector2d Bug1Algorithm::getRelativeVectorNormalized(Eigen::Vector2d fromVector, Eigen::Vector2d toVector) {
    Eigen::Vector2d relative = toVector - fromVector;
    relative.normalize();
    return relative;
}

double Bug1Algorithm::distance(Eigen::Vector2d point1, Eigen::Vector2d point2) {
    double distance = sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
    return distance;
}

double Bug1Algorithm::distanceToGoal(const amp::Problem2D& problem, Eigen::Vector2d point) {
    double distance = sqrt(pow(problem.q_goal[0] - point[0], 2) + pow(problem.q_goal[1] - point[1], 2));
    return distance;
}

// Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
// amp::Path2D path;
// path.waypoints.push_back(problem.q_init);
// path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
// path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
// path.waypoints.push_back(problem.q_goal);

// Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
// amp::Path2D path;
// Eigen::Vector2d currentPosition(0.989949, 0.989949);
// double xSearch;
// double ySearch;
// double searchAngle;
// path.waypoints.push_back(problem.q_init);
// path.waypoints.push_back(currentPosition);

// for (int i = 0; i < 180; i++) {
//     searchAngle = (2 * M_PI * i / 180);
//     xSearch = currentPosition[0] + (0.1 * cos(searchAngle));
//     ySearch = currentPosition[1] + (0.1 * sin(searchAngle));
//     Eigen::Vector2d nextSearch(xSearch, ySearch);

//     path.waypoints.push_back(nextSearch);
//     path.waypoints.push_back(currentPosition);
    // }
