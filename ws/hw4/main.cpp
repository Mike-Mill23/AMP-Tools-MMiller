// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "MinkowskiDifference.h"

// Include headers from hw4 ws
#include "MyLinkManipulator.h"


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
    ManipulatorState Ex2aConfig{M_PI / 6, M_PI / 3, 7 * M_PI / 4};
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
    
    }

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}



/*-------- Running Exercise 1 --------*/
// amp::Obstacle2D Ex1Triangle = HW4::getEx1TriangleObstacle();

// amp::Obstacle2D Ex1aCSpace = MinkowskiDifference(Ex1Triangle, Ex1Triangle);
// std::vector<amp::Polygon> polys{};
// polys.push_back(Ex1aCSpace);
// Visualizer::makeFigure(polys);

// std::vector<double> rotationAngles{};
// int numRotations = 12;
// std::vector<amp::Obstacle2D> Ex1bCSpace = MinkowskiDifference(Ex1Triangle, Ex1Triangle, rotationAngles, numRotations);
// Visualizer::makeFigure(Ex1bCSpace, rotationAngles);

// Visualizer::showFigures();


/*-------- Debugging Minkowski Difference --------*/
// amp::Obstacle2D Ex1Triangle = HW4::getEx1TriangleObstacle();
// amp::Obstacle2D rotatedEx1Tri = rotatePolygon(Ex1Triangle, (120 * M_PI / 180));

// amp::Obstacle2D Ex1bCSpace = MinkowskiDifference(rotatedEx1Tri, Ex1Triangle);

// std::vector<amp::Polygon> polys{};
// polys.push_back(Ex1Triangle);
// Visualizer::makeFigure(polys);
// polys.clear();
// polys.push_back(rotatedEx1Tri);
// Visualizer::makeFigure(polys);
// polys.clear();
// polys.push_back(Ex1bCSpace);
// Visualizer::makeFigure(polys);

// Visualizer::showFigures();
