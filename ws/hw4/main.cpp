// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "MinkowskiDifference.h"
#include "CollisionDetector.h"

// Include headers from hw4 ws
#include "MyLinkManipulator.h"
#include "MyGridCSpace.h"


using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    /*-------- Exercise 1 --------*/
    {
    amp::Obstacle2D Ex1Triangle = HW4::getEx1TriangleObstacle();

    amp::Obstacle2D Ex1aCSpace = MinkowskiDifference(Ex1Triangle, Ex1Triangle);
    std::vector<amp::Polygon> polys{};
    polys.push_back(Ex1aCSpace);
    Visualizer::makeFigure(polys);

    std::vector<double> rotationAngles{};
    int numRotations = 12;
    std::vector<amp::Obstacle2D> Ex1bCSpace = MinkowskiDifference(Ex1Triangle, Ex1Triangle, rotationAngles, numRotations);
    Visualizer::makeFigure(Ex1bCSpace, rotationAngles);

    Visualizer::showFigures();
    }

    /*-------- Exercise 2 --------*/
    {
    std::vector<double> Ex2aLinkLengths{0.5, 1.0, 0.5};
    ManipulatorState Ex2aConfig(3);
    Ex2aConfig << M_PI / 6, M_PI / 3, 7 * M_PI / 4;
    MyLinkManipulator planarManipulator1{Ex2aLinkLengths, Ex2aConfig};

    HW4::checkFK(planarManipulator1.getJointLocation(Ex2aConfig, 0), 0, planarManipulator1, Ex2aConfig);
    HW4::checkFK(planarManipulator1.getJointLocation(Ex2aConfig, 1), 1, planarManipulator1, Ex2aConfig);
    HW4::checkFK(planarManipulator1.getJointLocation(Ex2aConfig, 2), 2, planarManipulator1, Ex2aConfig);
    HW4::checkFK(planarManipulator1.getJointLocation(Ex2aConfig, 3), 3, planarManipulator1, Ex2aConfig);
    
    std::vector<double> Ex2bLinkLengths{1.0, 0.5, 1.0};
    Eigen::Vector2d Ex2bEndPos(2.0, 0.0);
    MyLinkManipulator planarManipulator2{Ex2bLinkLengths, Ex2bEndPos};
    }

    /*-------- Exercise 3 --------*/
    {
    std::vector<double> linkLengths{1.0, 1.0};
    ManipulatorState jointAngles(2);
    jointAngles << 0.0, 0.0;
    MyLinkManipulator planarManipulator{linkLengths, jointAngles};

    amp::Environment2D Ex3aEnv = HW4::getEx3Workspace1();
    MyGridCSpace2DConstructor gridCtor3a{};
    std::unique_ptr<amp::GridCSpace2D> ex3aGridSpace = gridCtor3a.construct(planarManipulator, Ex3aEnv);
    Visualizer::makeFigure(Ex3aEnv, planarManipulator, jointAngles);
    Visualizer::makeFigure(*ex3aGridSpace);

    amp::Environment2D Ex3bEnv = HW4::getEx3Workspace2();
    MyGridCSpace2DConstructor gridCtor3b{};
    std::unique_ptr<amp::GridCSpace2D> ex3bGridSpace = gridCtor3b.construct(planarManipulator, Ex3bEnv);
    Visualizer::makeFigure(Ex3bEnv, planarManipulator, jointAngles);
    Visualizer::makeFigure(*ex3bGridSpace);

    amp::Environment2D Ex3cEnv = HW4::getEx3Workspace3();
    MyGridCSpace2DConstructor gridCtor3c{};
    std::unique_ptr<amp::GridCSpace2D> ex3cGridSpace = gridCtor3c.construct(planarManipulator, Ex3cEnv);
    Visualizer::makeFigure(Ex3cEnv, planarManipulator, jointAngles);
    Visualizer::makeFigure(*ex3cGridSpace);

    Visualizer::showFigures();
    }

    // Grade method
    MyGridCSpace2DConstructor gridCtor{};
    amp::HW4::grade<MyLinkManipulator>(gridCtor, "michael.miller-5@colorado.edu", argc, argv);
    return 0;
}
