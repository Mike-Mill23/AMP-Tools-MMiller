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
    // {
    //     double xi{1.0};
    //     double eta{1.0};
    //     double epsilon{0.25};
    //     double alpha{0.025};
    //     double beta{0.8};

    //     amp::Problem2D ex2aEnv = HW5::getWorkspace1();
    //     // ex2aEnv.print();

    //     amp::Path2D path{};
    //     MyGDAlgorithm gradAlgo{xi, eta, epsilon, alpha, beta};

    //     path = gradAlgo.plan(ex2aEnv);
    //     path.print();
    //     LOG("Path Length: " << path.length());

    //     Visualizer::makeFigure(ex2aEnv, path);
    //     Visualizer::showFigures();
    // }

    /*-------- Exercise 2b --------*/
    {
        double xi{1.0};
        double eta{0.8};
        double epsilon{0.25};
        double alpha{0.001};
        double beta1{0.8};
        double beta2{0.9};

        amp::Problem2D ex2bEnv1 = HW2::getWorkspace1();

        // MyPotentialFunction uFunction{xi, eta, epsilon, ex2bEnv1};
        // LOG("uFunction(q_init): " << uFunction(ex2bEnv1.q_init));
        // LOG("uFunction(q_goal): " << uFunction(ex2bEnv1.q_goal));
        // LOG("uFunction(3.5, 0.5): " << uFunction(Eigen::Vector2d(3.5, 0.5)));
        // LOG("uFunction(4.5, 0.5): " << uFunction(Eigen::Vector2d(4.5, 0.5)));
        // LOG("uFunction(4.5, 1.5): " << uFunction(Eigen::Vector2d(4.5, 1.5)));
        // LOG("uFunction(3.5, 1.5): " << uFunction(Eigen::Vector2d(3.5, 1.5)));
        // LOG("uFunction(4.0, 0.4): " << uFunction(Eigen::Vector2d(4.0, 0.4)));
        // LOG("uFunction(4.0, 0.0): " << uFunction(Eigen::Vector2d(4.0, 0.0)));
        // Visualizer::makeFigure(uFunction, ex2bEnv1.x_min, ex2bEnv1.x_max, ex2bEnv1.y_min, ex2bEnv1.y_max, 100);

        amp::Path2D path{};
        MyGDAlgorithm gradAlgo{xi, eta, epsilon, alpha, beta1, beta2};

        path = gradAlgo.plan(ex2bEnv1);
        path.print();

        Visualizer::makeFigure(ex2bEnv1, path);

        // amp::Problem2D ex2bEnv2 = HW2::getWorkspace2();
        // Visualizer::makeFigure(ex2bEnv2);
        Visualizer::showFigures();
    }
    
    return 0;
}


/*-------- Debugging --------*/

// ---- findClosestCs() ---- //
// Eigen::Vector2d q(4.0, 0.0);
// std::vector<Eigen::Vector2d> listCs = gradAlgo.findClosestCs(q, ex2aEnv.obstacles.front());

// for (int i = 0; i < listCs.size(); i++) {
//     LOG("Side " << i << " c:\n" << listCs[i]);
// }

// ---- d_iq() ---- //
// Eigen::Vector2d c{};
// double d0init = gradAlgo.d_iq(ex2aEnv.q_init, ex2aEnv.obstacles.back(), c);
// LOG("d_0(q_init): " << d0init);
// LOG("c:\n" << c);

