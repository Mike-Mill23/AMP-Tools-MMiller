#include "MyBugAlgorithm.h"

using namespace amp;


amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {
    bool success;
    LOG("Starting Plan " << bug.planCount);

    bug.problem = problem;
    bug.path.waypoints.push_back(bug.problem.q_init);

    while (!HW2::check(bug.path, bug.problem, false)) {
        move.moveToGoal = true;
        success = step();

        if (!success) {
            move.moveToGoal = false;
            followObstacle();
            gotoLeavePoint();
        }

        bug.collisions.clear();
        bug.splitPath.fill(amp::Path2D());
    }

    LOG("Ending Plan " << bug.planCount);
    bug.planCount++;
    return bug.path;
}

bool Bug1Algorithm::step() {
    move.currentPosition = bug.path.waypoints.back();
    bug.tmpCollisions.clear();
    bool success = false;
    bool isCollision;

    // LOG("Current Position:\n" << move.currentPosition);

    if (move.moveToGoal) {
        move.rise = bug.problem.q_goal[1] - move.currentPosition[1];
        move.run = bug.problem.q_goal[0] - move.currentPosition[0];
        move.hyp = sqrt(pow(move.rise, 2) + pow(move.run, 2));
    } else {
        move.rise = move.directionTravel[1];
        move.run = move.directionTravel[0];
        move.hyp = move.directionTravel.norm();
    }

    if (move.dist = distanceToGoal(bug.problem, move.currentPosition) <= bug.stepSize) {
        move.nextPosition = bug.problem.q_goal;
    } else {
        move.nextPosition[0] = move.currentPosition[0] + (bug.stepSize / move.hyp) * move.run;
        move.nextPosition[1] = move.currentPosition[1] + (bug.stepSize / move.hyp) * move.rise;
    }

    // LOG("Next Position:\n" << move.nextPosition);

    move.checkPath = bug.path;
    move.checkPath.waypoints.push_back(move.nextPosition);
    move.checkPath.waypoints.push_back(bug.problem.q_goal);

    HW2::check(move.checkPath, bug.problem, bug.tmpCollisions, false);
    isCollision = isTrueCollision();


    if (isCollision) {
        bug.collisions.push_back(bug.tmpCollisions.front());
        if (bug.collisions.size() > 2) {
            bug.collisions.erase(bug.collisions.begin());
        }
    } else {
        success = true;
        bug.path.waypoints.push_back(move.nextPosition);
    }

    bug.tmpCollisions.clear();
    move.currentPosition = bug.path.waypoints.back();

    return success;
}

void Bug1Algorithm::followObstacle() {
    // LOG("-------- followObstacle --------");
    bug.hitPoint = bug.path.waypoints.back();
    bug.leavePoint = bug.path.waypoints.back();
    move.directionWall = getRelativeVectorNormalized(bug.hitPoint, bug.collisions.back());
    move.directionTravel = move.directionWall;
    move.prevDirectionTravel = Eigen::Vector2d(0.0, 0.0);
    int stepCount = 1;
    bool searched = false;

    do {
        find.startAngle = atan2(move.directionWall[1], move.directionWall[0]);
        
        while (!searchSurroundings()) {
            if (!searched) {
                searched = true;
                bug.path.waypoints.pop_back();
            }
            double distanceToCollision = distance(bug.path.waypoints.back(), bug.collisions.back());
            move.directionTravel = getRelativeVectorNormalized(bug.path.waypoints.back(), bug.collisions.back());
            move.prevDirectionTravel = Eigen::Vector2d(0.0, 0.0);
            find.startAngle = atan2(move.directionTravel[1], move.directionTravel[0]);
            bug.tmpStepSize = bug.stepSize;
            bug.stepSize = distanceToCollision / 2;
            step();
            bug.stepSize = bug.tmpStepSize;
        }
        searched = false;

        while (!step()) {
            double currentAngle = atan2(move.directionTravel[1], move.directionTravel[0]);
            move.prevDirectionTravel = Eigen::Vector2d(0.0, 0.0);
            move.tmpDirectionTravel = move.directionTravel;
            move.directionTravel[0] = cos(currentAngle + (stepCount * M_PI / 2));
            move.directionTravel[1] = sin(currentAngle + (stepCount * M_PI / 2));
            step();
            move.directionTravel = move.tmpDirectionTravel;
            stepCount++;
        }
        stepCount = 1;

        if (distanceToGoal(bug.problem, bug.path.waypoints.back()) < distanceToGoal(bug.problem, bug.leavePoint)) {
            if (bug.splitPath[1].waypoints.size() > 0) {
                bug.splitPath[0].waypoints.insert(bug.splitPath[0].waypoints.end(), bug.splitPath[1].waypoints.begin(), 
                                                  bug.splitPath[1].waypoints.end());
                bug.splitPath[1].waypoints.clear();
            }
            bug.leavePoint = bug.path.waypoints.back();
            bug.splitPath[0].waypoints.push_back(bug.leavePoint);
        } else {
            bug.splitPath[1].waypoints.push_back(bug.path.waypoints.back());
        }

        move.directionWall = getRelativeVectorNormalized(bug.path.waypoints.back(), bug.collisions.back());
    } while (sqrt(pow(bug.path.waypoints.back()[0] - bug.hitPoint[0],2) + 
                  pow(bug.path.waypoints.back()[1] - bug.hitPoint[1],2)) >= (0.95 * bug.stepSize));

    bug.path.waypoints.push_back(bug.hitPoint);

    return;
}

bool Bug1Algorithm::searchSurroundings() {
    find.pathSize = bug.path.waypoints.size();
    find.searchRay = bug.path;
    find.isCollision = false;
    find.prevIsCollision = false;
    find.startWithCollision = false;
    find.edgePoints.clear();

    for (int i = 0; i < find.numSearchPoints; i++) {
        find.searchAngle = (2 * M_PI * i / find.numSearchPoints) + find.startAngle;
        move.nextPosition[0] = move.currentPosition[0] + (bug.stepSize * cos(find.searchAngle));
        move.nextPosition[1] = move.currentPosition[1] + (bug.stepSize * sin(find.searchAngle));

        find.searchRay.waypoints.push_back(move.nextPosition);
        find.searchRay.waypoints.push_back(bug.problem.q_goal);
        bug.tmpCollisions.clear();
        HW2::check(find.searchRay, bug.problem, bug.tmpCollisions, false);
        find.isCollision = isTrueCollision();

        if (find.isCollision && move.directionTravel.dot(move.prevDirectionTravel) >= DIRECTION_EPSILON) {
            bug.collisions.push_back(bug.tmpCollisions.front());
            if (bug.collisions.size() > 2) {
                bug.collisions.erase(bug.collisions.begin());
            }
            break;
        }

        if (i == 0) {
            find.startWithCollision = find.isCollision;
        }

        if (find.startWithCollision) {
            if (find.isCollision && find.prevIsCollision) {
                if (find.edgePoints.size() > 0) {
                    find.edgePoints[find.edgePoints.size() - 1] = bug.tmpCollisions.front();
                }
            } else if (find.isCollision && !find.prevIsCollision) {
                find.edgePoints.push_back(bug.tmpCollisions.front());
                if (find.edgePoints.size() == 2) {
                    calculateDirectionTravel();
                    break;
                }
            }
        } else {
            if (find.edgePoints.size() == 2) {
                if (find.isCollision) {
                    find.edgePoints[find.edgePoints.size() - 1] = bug.tmpCollisions.front();
                } else {
                    calculateDirectionTravel();
                    find.isCollision = find.prevIsCollision;
                    break;
                }
            } else {
                if (find.isCollision) {
                    find.edgePoints.push_back(bug.tmpCollisions.front());
                }
            }
        }

        find.prevIsCollision = find.isCollision;
        find.searchRay.waypoints.erase(find.searchRay.waypoints.begin() + find.pathSize, find.searchRay.waypoints.end());
    }

    move.prevDirectionTravel = move.directionTravel;
    bug.tmpCollisions.clear();
    return find.isCollision;
}

void Bug1Algorithm::calculateDirectionTravel() {
    checker.toPoint1 = getRelativeVectorNormalized(move.currentPosition, find.edgePoints.front());
    checker.toPoint2 = getRelativeVectorNormalized(move.currentPosition, find.edgePoints.back());

    checker.dotProdPoint1 = move.directionTravel.dot(checker.toPoint1);
    checker.dotProdPoint2 = move.directionTravel.dot(checker.toPoint2);

    checker.checkLeftPt1 = (move.directionTravel[0] * checker.toPoint1[1]) - (move.directionTravel[1] * checker.toPoint1[0]);
    checker.checkLeftPt2 = (move.directionTravel[0] * checker.toPoint2[1]) - (move.directionTravel[1] * checker.toPoint2[0]);

    if (checker.checkLeftPt1 * checker.checkLeftPt2 < 0) {   // Different sides of directionTravel
        if (checker.checkLeftPt1 >= 0) {
            move.directionTravel = getRelativeVectorNormalized(checker.toPoint2, checker.toPoint1);
            bug.collisions.clear();
            bug.collisions.push_back(find.edgePoints.back());
            bug.collisions.push_back(find.edgePoints.front());
        } else {
            move.directionTravel = getRelativeVectorNormalized(checker.toPoint1, checker.toPoint2);
            bug.collisions.clear();
            bug.collisions.push_back(find.edgePoints.front());
            bug.collisions.push_back(find.edgePoints.back());
        }
    } else {                                                 // On the same side of directionTravel
        if (checker.dotProdPoint1 > checker.dotProdPoint2) {
            move.directionTravel = getRelativeVectorNormalized(checker.toPoint2, checker.toPoint1);
            bug.collisions.clear();
            bug.collisions.push_back(find.edgePoints.back());
            bug.collisions.push_back(find.edgePoints.front());
        } else {
            move.directionTravel = getRelativeVectorNormalized(checker.toPoint1, checker.toPoint2);
            bug.collisions.clear();
            bug.collisions.push_back(find.edgePoints.front());
            bug.collisions.push_back(find.edgePoints.back());
        }
    }

    return;
}

void Bug1Algorithm::gotoLeavePoint() {
    double distanceLeft = bug.splitPath[0].length();
    double distanceRight = bug.splitPath[1].length();

    if (distanceRight < distanceLeft) {
        bug.path.waypoints.insert(bug.path.waypoints.end(), bug.splitPath[1].waypoints.rbegin(), 
                                  bug.splitPath[1].waypoints.rend());
    } else {
        bug.path.waypoints.insert(bug.path.waypoints.end(), bug.splitPath[0].waypoints.begin(), 
                                  bug.splitPath[0].waypoints.end());
    }

    bug.path.waypoints.push_back(bug.leavePoint);

    return;
}

bool Bug1Algorithm::isTrueCollision() {
    bool isTrueCollision;

    if (bug.tmpCollisions.size() > 0) {
        checker.currentToNext = getRelativeVectorNormalized(move.currentPosition, move.nextPosition);
        checker.currentToCollision = getRelativeVectorNormalized(move.currentPosition, bug.tmpCollisions.front());
        checker.dist = distance(move.currentPosition, bug.tmpCollisions.front());
        checker.direction = checker.currentToNext.dot(checker.currentToCollision);
    } else {
        checker.dist = distanceToGoal(bug.problem, move.currentPosition);
        checker.direction = 0.0;
    }

    if (checker.dist <= (1.1 * bug.stepSize) && checker.direction >= DIRECTION_EPSILON) {
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

void Bug1Algorithm::clear() {
    bug.stepSize = 0.1;
    bug.tmpStepSize = 0.0;
    bug.problem = amp::Problem2D();
    bug.path = amp::Path2D();
    bug.collisions.clear();
    bug.tmpCollisions.clear();
    bug.hitPoint = Eigen::Vector2d(0.0, 0.0);
    bug.leavePoint = Eigen::Vector2d(0.0, 0.0);
    bug.splitPath.fill(amp::Path2D());
    bug.planCount = 1;

    move.checkPath = amp::Path2D();
    move.currentPosition = Eigen::Vector2d(0.0, 0.0);
    move.nextPosition = Eigen::Vector2d(0.0, 0.0);
    move.directionWall = Eigen::Vector2d(0.0, 0.0);
    move.directionTravel = Eigen::Vector2d(0.0, 0.0);
    move.prevDirectionTravel = Eigen::Vector2d(0.0, 0.0);
    move.tmpDirectionTravel = Eigen::Vector2d(0.0, 0.0);
    move.dist = 0.0;
    move.rise = 0.0;
    move.run = 0.0;
    move.hyp = 0.0;
    move.moveToGoal = true;

    find.pathSize = 0;
    find.searchRay = amp::Path2D();
    find.startAngle = 0.0;
    find.searchAngle = 0.0;
    find.dist = 0.0;
    find.direction = 0.0;
    find.isCollision = false;
    find.prevIsCollision = false;
    find.startWithCollision = false;
    find.edgePoints.clear();

    checker.toPoint1 = Eigen::Vector2d(0.0, 0.0);
    checker.toPoint2 = Eigen::Vector2d(0.0, 0.0);
    checker.currentToNext = Eigen::Vector2d(0.0, 0.0);
    checker.currentToCollision = Eigen::Vector2d(0.0, 0.0);
    checker.dotProdPoint1 = 0.0;
    checker.dotProdPoint2 = 0.0;
    checker.checkLeftPt1 = 0.0;
    checker.checkLeftPt2 = 0.0;
    checker.dist = 0.0;
    checker.direction = 0.0;

    return;
}
