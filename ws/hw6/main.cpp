// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW6.h"
#include "hw/HW4.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "CollisionDetector.h"
#include "MyGridCSpace.h"
#include "MyLinkManipulator.h"

// Include headers from hw6 ws
#include "MyWaveFrontAlgorithm.h"
#include "MyManipulatorWaveFrontAlgorithm.h"


using namespace amp;

int main(int argc, char** argv) {

    /*-------- Exercise 1 WS 1 --------*/
    {
        // MyPointWaveFrontAlgorithm pointPlanner{};
        // amp::Problem2D Ex1Ws1 = HW2::getWorkspace1();
        // std::unique_ptr<amp::GridCSpace2D> gridCSpace = pointPlanner.constructDiscretizedWorkspace(Ex1Ws1);
        // amp::Path2D path = pointPlanner.plan(Ex1Ws1);
        // LOG("Path length: " << path.length());
        // Visualizer::makeFigure(Ex1Ws1, path);
        // Visualizer::makeFigure(*gridCSpace);
        // Visualizer::showFigures();
    }

    /*-------- Exercise 1 WS 2 --------*/
    {
        // MyPointWaveFrontAlgorithm pointPlanner{};
        // amp::Problem2D Ex1Ws2 = HW2::getWorkspace2();
        // std::unique_ptr<amp::GridCSpace2D> gridCSpace = pointPlanner.constructDiscretizedWorkspace(Ex1Ws2);
        // amp::Path2D path = pointPlanner.plan(Ex1Ws2);
        // LOG("Path length: " << path.length());
        // Visualizer::makeFigure(Ex1Ws2, path);
        // Visualizer::makeFigure(*gridCSpace);
        // Visualizer::showFigures();
    }

    /*-------- Exercise 2 Ws 1 --------*/
    {
        amp::Problem2D Ex2Ws1 = HW6::getHW4Problem1();
        std::vector<double> linkLengths{1.0, 1.0};
        Eigen::Vector2d endEffectLoc{-2.0, 0.0};
        MyLinkManipulator manipulator{linkLengths, endEffectLoc};
        std::shared_ptr<MyGridCSpace2DConstructor> gridCtor = std::make_shared<MyGridCSpace2DConstructor>();
        std::unique_ptr<GridCSpace2D> cSpace = gridCtor->construct(manipulator, Ex2Ws1);
        MyManipulatorWaveFrontAlgorithm manipulatorPlanner{gridCtor};
        amp::Path2D path = manipulatorPlanner.plan(manipulator, Ex2Ws1);
        Visualizer::makeFigure(*cSpace, path);
        Visualizer::makeFigure(Ex2Ws1, manipulator, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 2 Ws 2 --------*/
    {
        amp::Problem2D Ex2Ws2 = HW6::getHW4Problem2();
        std::vector<double> linkLengths{1.0, 1.0};
        Eigen::Vector2d endEffectLoc{-2.0, 0.0};
        MyLinkManipulator manipulator{linkLengths, endEffectLoc};
        std::shared_ptr<MyGridCSpace2DConstructor> gridCtor = std::make_shared<MyGridCSpace2DConstructor>();
        std::unique_ptr<GridCSpace2D> cSpace = gridCtor->construct(manipulator, Ex2Ws2);
        MyManipulatorWaveFrontAlgorithm manipulatorPlanner{gridCtor};
        amp::Path2D path = manipulatorPlanner.plan(manipulator, Ex2Ws2);
        Visualizer::makeFigure(*cSpace, path);
        Visualizer::makeFigure(Ex2Ws2, manipulator, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 2 Ws 3 --------*/
    {
        amp::Problem2D Ex2Ws3 = HW6::getHW4Problem3();
        std::vector<double> linkLengths{1.0, 1.0};
        Eigen::Vector2d endEffectLoc{-2.0, 0.0};
        MyLinkManipulator manipulator{linkLengths, endEffectLoc};
        std::shared_ptr<MyGridCSpace2DConstructor> gridCtor = std::make_shared<MyGridCSpace2DConstructor>();
        std::unique_ptr<GridCSpace2D> cSpace = gridCtor->construct(manipulator, Ex2Ws3);
        MyManipulatorWaveFrontAlgorithm manipulatorPlanner{gridCtor};
        amp::Path2D path = manipulatorPlanner.plan(manipulator, Ex2Ws3);
        Visualizer::makeFigure(*cSpace, path);
        Visualizer::makeFigure(Ex2Ws3, manipulator, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 3 --------*/
    {
        
    }

    // Grade method
    // amp::HW6::grade(gradAlgo, "michael.miller-5@colorado.edu", argc, argv);
    
    return 0;
}
