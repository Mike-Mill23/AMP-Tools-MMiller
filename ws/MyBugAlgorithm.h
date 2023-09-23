#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <array>

#define _USE_MATH_DEFINES
#include <math.h>

#define DIRECTION_EPSILON cos(M_PI / 180)   // One degree epsilon for direction check

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1Algorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        /// @brief Step in the environment, set collisions
        /// @return true/false if collision
        bool step(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d>& collisions, double stepSize);

        /// @brief Step in the environment, set collisions
        /// @return true/false if collision
        bool step(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d>& collisions, double stepSize, 
                  const double rise, const double run, const double hyp);

        void followObstacle(const amp::Problem2D& problem, amp::Path2D& path, std::vector<Eigen::Vector2d>& collisions, 
                            double stepSize, Eigen::Vector2d& hitPoint, Eigen::Vector2d& leavePoint, 
                            std::array<amp::Path2D, 2>& splitPath);

        bool searchSurroundings(const amp::Problem2D& problem, amp::Path2D& path, Eigen::Vector2d& directionTravel, 
                    Eigen::Vector2d& currentPosition, std::vector<Eigen::Vector2d>& searchCollisionList, double stepSize, 
                    const double startAngle = 0);

        void calculateDirectionTravel(Eigen::Vector2d& currentPosition, Eigen::Vector2d& directionTravel, 
                                      std::vector<Eigen::Vector2d>& searchCollisionList, std::vector<Eigen::Vector2d>& edgePoints);

        void gotoLeavePoint(amp::Path2D& path, Eigen::Vector2d& hitPoint, Eigen::Vector2d& leavePoint, 
                            std::array<amp::Path2D, 2>& splitPath);

        bool isTrueCollision(const amp::Problem2D& problem, std::vector<Eigen::Vector2d>& collisionList, 
                              Eigen::Vector2d& currentPosition, Eigen::Vector2d& nextPosition, double stepSize);

        Eigen::Vector2d getRelativeVectorNormalized(Eigen::Vector2d fromVector, Eigen::Vector2d toVector);

        double distance(Eigen::Vector2d point1, Eigen::Vector2d point2);

        double distanceToGoal(const amp::Problem2D& problem, Eigen::Vector2d point);
    
    private:
        // Add any member variables here...
};
