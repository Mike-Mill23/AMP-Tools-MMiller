#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"
#include "CollisionDetector.h"

#include <random>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyGDAlgorithm : public GDAlgorithm {
        public:
            MyGDAlgorithm() = default;
            MyGDAlgorithm(double xi, double eta, double epsilon, double alpha) : xi(xi), eta(eta), epsilon(epsilon), alpha(alpha) {}
            ~MyGDAlgorithm() = default;

            /// @brief Solve a motion planning problem.
            /// @param problem Motion planning problem
            /// @return Path solution of the point agent
            amp::Path2D plan(const amp::Problem2D& problem);

        private:
            void setStarParams(const amp::Problem2D& problem);
            Eigen::Vector2d gradU(const Eigen::Vector2d& q, const amp::Problem2D& problem);
            Eigen::Vector2d gradUatt(const Eigen::Vector2d& q, const amp::Problem2D& problem);
            Eigen::Vector2d gradUrepi(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, int index);
            double d_iq(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, Eigen::Vector2d& c);
            std::vector<Eigen::Vector2d> findClosestCs(const Eigen::Vector2d& q, const amp::Obstacle2D& obst);
            double distanceL2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

            double xi{1.0};
            double eta{1.0};
            double epsilon{0.25};
            double alpha{1.0};
            double d_star_goal{1.0};
            std::vector<double> Q_star_i{1, 1.0};
    };
}
