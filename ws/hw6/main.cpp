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
#include "MyAStar.h"

// Include other stdlib headers
#include <iostream>


using namespace amp;

int main(int argc, char** argv) {

    /*-------- Exercise 1 WS 1 --------*/
    {
        MyPointWaveFrontAlgorithm pointPlanner{};
        amp::Problem2D Ex1Ws1 = HW2::getWorkspace1();
        std::unique_ptr<amp::GridCSpace2D> gridCSpace = pointPlanner.constructDiscretizedWorkspace(Ex1Ws1);
        amp::Path2D path = pointPlanner.plan(Ex1Ws1);
        LOG("Path length: " << path.length());
        Visualizer::makeFigure(Ex1Ws1, path);
        Visualizer::makeFigure(*gridCSpace);
        Visualizer::showFigures();
        HW6::checkPointAgentPlan(path, Ex1Ws1);
    }

    /*-------- Exercise 1 WS 2 --------*/
    {
        MyPointWaveFrontAlgorithm pointPlanner{};
        amp::Problem2D Ex1Ws2 = HW2::getWorkspace2();
        std::unique_ptr<amp::GridCSpace2D> gridCSpace = pointPlanner.constructDiscretizedWorkspace(Ex1Ws2);
        amp::Path2D path = pointPlanner.plan(Ex1Ws2);
        LOG("Path length: " << path.length());
        Visualizer::makeFigure(Ex1Ws2, path);
        Visualizer::makeFigure(*gridCSpace);
        Visualizer::showFigures();
        HW6::checkPointAgentPlan(path, Ex1Ws2);
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
        HW6::checkLinkManipulatorPlan(path, manipulator, Ex2Ws1);
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
        HW6::checkLinkManipulatorPlan(path, manipulator, Ex2Ws2);
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
        HW6::checkLinkManipulatorPlan(path, manipulator, Ex2Ws3);
        Visualizer::makeFigure(Ex2Ws3, manipulator, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 3 --------*/
    {
        amp::ShortestPathProblem Ex3Graph = HW6::getEx3SPP();
        amp::LookupSearchHeuristic Ex3Heuristic = HW6::getEx3Heuristic();
        MyAStar aStarAlgo{};
        MyAStar::GraphSearchResult result = aStarAlgo.search(Ex3Graph, Ex3Heuristic);
        LOG("Found Goal?: " << result.success);
        LOG("Path Cost: " << result.path_cost);
        LOG("Node Path: ");
        for (auto n : result.node_path) {
            std::cout << n << " ";
        }
        std::cout << std::endl;
    }

    /*-------- Grading --------*/
    {
        MyPointWaveFrontAlgorithm pointPlanner{};

        std::shared_ptr<MyGridCSpace2DConstructor> gridCtor = std::make_shared<MyGridCSpace2DConstructor>();
        MyManipulatorWaveFrontAlgorithm manipulatorPlanner{gridCtor};

        MyAStar aStarAlgo{};

        amp::HW6::grade(pointPlanner, manipulatorPlanner, aStarAlgo, "michael.miller-5@colorado.edu", argc, argv);
    }
    
    return 0;
}
