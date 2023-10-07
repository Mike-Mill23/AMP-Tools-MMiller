#pragma once

#include "AMPCore.h"
#include <Eigen/Geometry>
#include "hw/HW4.h"
#include "MinkowskiDifference.h"
#include "CollisionDetector.h"
#include <iostream>

#define CELL_SIZE 0.01
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyGridCSpace2DConstructor : public GridCSpace2DConstructor {
        public:
            /// @brief Construct a CSpace from a manipulator and an environment
            /// @param manipulator Two link manipulator (consider ussing `ASSERT` to make sure the manipulator is 2D)
            /// @param env Environment
            /// @return Unique pointer to your C-space. 
            /// NOTE: We use a unique pointer here to be able to move the C-space without copying it, since grid discretization
            /// C-spaces can contain a LOT of memory, so copying would be a very expensive operation. Additionally, a pointer is polymorphic
            /// which allows the type to pose as a GridCSpace2D (even though GridCSpace2D is abstract)
            std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env);
        
            MyGridCSpace2DConstructor() = default;
            ~MyGridCSpace2DConstructor() = default;
    };

    class MyGridCSpace2D : public GridCSpace2D{
        public:
            MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
                         : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}
            
            /******* User Implemented Methods ********/

            /// @brief Access the C-space with continuous variables (interpolation between cells)
            /// @param x0 Value of the first configuration variable
            /// @param x1 Value of the second configuration variable
            /// @return `true` if the the point is in collision, `false` if it is not
            bool inCollision(double x0, double x1) const;

            /*****************************************/

            ~MyGridCSpace2D() = default;
    };
}