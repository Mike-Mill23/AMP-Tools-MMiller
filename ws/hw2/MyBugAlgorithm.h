#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <array>

#define _USE_MATH_DEFINES
#include <math.h>

#define DIRECTION_EPSILON cos(2 * M_PI / 180)   // Three degrees epsilon for direction check

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class Bug1Algorithm : public amp::BugAlgorithm {
    public:
        // Bug1Algorithm(const amp::Path2D& problem) : bug.problem(problem) {};

        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        /// @brief Step in the environment, set collisions
        /// @return true/false if collision
        bool step();

        void followObstacle();

        bool searchSurroundings();

        void calculateDirectionTravel();

        void gotoLeavePoint();

        bool isTrueCollision();

        Eigen::Vector2d getRelativeVectorNormalized(Eigen::Vector2d fromVector, Eigen::Vector2d toVector);

        double distance(Eigen::Vector2d point1, Eigen::Vector2d point2);

        double distanceToGoal(const amp::Problem2D& problem, Eigen::Vector2d point);

        void clear();
    
    private:
        // Add any member variables here...
        struct planning {
            double stepSize{0.1};
            double tmpStepSize{0.0};
            amp::Problem2D problem{};
            amp::Path2D path{};
            std::vector<Eigen::Vector2d> collisions{};
            std::vector<Eigen::Vector2d> tmpCollisions{};
            Eigen::Vector2d hitPoint{};
            Eigen::Vector2d leavePoint{};
            std::array<amp::Path2D, 2> splitPath{};
            int planCount{1};
        } bug;

        struct stepping {
            amp::Path2D checkPath{};
            Eigen::Vector2d currentPosition{};
            Eigen::Vector2d nextPosition{};
            Eigen::Vector2d directionWall{};
            Eigen::Vector2d directionTravel{};
            Eigen::Vector2d prevDirectionTravel{};
            Eigen::Vector2d tmpDirectionTravel{};
            double dist{0.0};
            double rise{0.0};
            double run{0.0};
            double hyp{0.0};
            bool moveToGoal{true};
        } move;

        struct searching {
            const int numSearchPoints{360 / 2};
            int pathSize{0};
            amp::Path2D searchRay{};
            double startAngle{0.0};
            double searchAngle{0.0};
            double dist{0.0};
            double direction{0.0};
            bool isCollision{false};
            bool prevIsCollision{false};
            bool startWithCollision{false};
            std::vector<Eigen::Vector2d> edgePoints{};
        } find;

        struct checking {
            Eigen::Vector2d toPoint1{};
            Eigen::Vector2d toPoint2{};
            Eigen::Vector2d currentToNext{};
            Eigen::Vector2d currentToCollision{};
            double dotProdPoint1{0.0};
            double dotProdPoint2{0.0};
            double checkLeftPt1{0.0};
            double checkLeftPt2{0.0};
            double dist{0.0};
            double direction{0.0};
        } checker;
};
