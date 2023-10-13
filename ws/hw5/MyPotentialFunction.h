#pragma once

#include "AMPCore.h"
#include "hw/HW5.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyPotentialFunction : public PotentialFunction2D {
        public:
            MyPotentialFunction() = default;
            MyPotentialFunction(double xi, double eta, double epsilon, amp::Problem2D problem);
            ~MyPotentialFunction() = default;

            /******* User Implemented Methods ********/

            /// @brief Calling operator. Get the value of the potential function at the coordinate `q`.
            /// @param q Coordinate in 2D space to evaluate the potential function value at
            /// @return The potential function value (height)
            double operator()(const Eigen::Vector2d& q) const { return U(q); }

            /*****************************************/

        private:
            double U(const Eigen::Vector2d& q) const;
            double Uatt(const Eigen::Vector2d& q) const;
            double Urepi(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, int index) const;
            double d_iq(const Eigen::Vector2d& q, const amp::Obstacle2D& obst, Eigen::Vector2d& c) const;
            std::vector<Eigen::Vector2d> findClosestCs(const Eigen::Vector2d& q, const amp::Obstacle2D& obst) const;
            double distanceL2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;

            double xi{1.0};
            double eta{1.0};
            double epsilon{0.25};
            double d_star_goal{1.0};
            std::vector<double> Q_star_i{1, 1.0};
            amp::Problem2D problem{};
    };
}