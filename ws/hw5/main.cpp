// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "CollisionDetector.h"

// Include headers from hw4 ws
#include "MyGDAlgorithm.h"
#include "MyPotentialFunction.h"


using namespace amp;

int main(int argc, char** argv) {

    /*-------- Exercise 2a --------*/
    {
        double xi{1.0};
        double eta{0.8};
        double epsilon{0.25};
        double alpha{0.01};

        amp::Problem2D ex2aEnv = HW5::getWorkspace1();
        // ex2aEnv.print();

        amp::Path2D path{};
        MyGDAlgorithm gradAlgo{xi, eta, epsilon, alpha};

        path = gradAlgo.plan(ex2aEnv);
        // path.print();
        LOG("Path Length: " << path.length());

        Visualizer::makeFigure(ex2aEnv, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 2b WS 1 --------*/
    {
        double xi{1.0};
        double eta{0.8};
        double epsilon{0.25};
        double alpha{0.01};

        amp::Problem2D ex2bEnv1 = HW2::getWorkspace1();

        amp::Path2D path{};
        MyGDAlgorithm gradAlgo{xi, eta, epsilon, alpha};

        path = gradAlgo.plan(ex2bEnv1);
        // path.print();
        LOG("Path Length: " << path.length());

        Visualizer::makeFigure(ex2bEnv1, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 2b WS 2 --------*/
    {
        double xi{1.0};
        double eta{0.8};
        double epsilon{0.25};
        double alpha{0.01};

        amp::Problem2D ex2bEnv2 = HW2::getWorkspace2();

        amp::Path2D path{};
        MyGDAlgorithm gradAlgo{xi, eta, epsilon, alpha};

        path = gradAlgo.plan(ex2bEnv2);
        // path.print();
        LOG("Path Length: " << path.length());

        Visualizer::makeFigure(ex2bEnv2, path);
        Visualizer::showFigures();
    }

    // Grade method
    double xi{1.0};
    double eta{0.8};
    double epsilon{0.25};
    double alpha{0.01};
    MyGDAlgorithm gradAlgo{xi, eta, epsilon, alpha};
    amp::HW5::grade(gradAlgo, "michael.miller-5@colorado.edu", argc, argv);
    
    return 0;
}
