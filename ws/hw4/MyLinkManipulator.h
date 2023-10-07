#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include "MinkowskiDifference.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {

    class MyLinkManipulator : public LinkManipulator2D {
        public:
            MyLinkManipulator() = default;
            MyLinkManipulator(const std::vector<double>& linkLengths, 
                              const ManipulatorState& config, 
                              const Eigen::Vector2d& baseLocation = Eigen::Vector2d(0.0, 0.0));
            MyLinkManipulator(const std::vector<double>& linkLengths, 
                              const Eigen::Vector2d& endEffectPos, 
                              const Eigen::Vector2d& baseLocation = Eigen::Vector2d(0.0, 0.0));
            ~MyLinkManipulator();

            /******* User Implemented Methods ********/

            /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
            /// @param state Joint angle state (radians). Must have size() == nLinks()
            /// @param joint_index Joint index in order of base to end effector 
            /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
            /// @return Joint coordinate
            Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const;

            /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
            /// @param end_effector_location End effector coordinate
            /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
            ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const;

            /*****************************************/

        private:
            Eigen::Matrix<double, 3, 3> getTransformationMatrix(const double& rotation, 
                                                                const double& translation) const;
            double getEndEffectorDistance(const Eigen::Vector2d& jointLocation, 
                                          const Eigen::Vector2d& endEffectorLocation) const;
            double getEndEffectorAngle(const Eigen::Vector2d& jointLocation, 
                                       const Eigen::Vector2d& endEffectorLocation) const;

            std::vector<Eigen::Vector2d> jointLocations{};
    };

}
