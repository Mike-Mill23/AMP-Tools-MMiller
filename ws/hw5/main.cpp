// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "CollisionDetector.h"

// Include headers from hw4 ws
#include "MyGDAlgorithm.h"


using namespace amp;

int main(int argc, char** argv) {

    /*-------- Exercise 2a --------*/
    {
        amp::Problem2D ex2aEnv = HW5::getWorkspace1();
        // ex2aEnv.print();
        MyGDAlgorithm gradAlgo{};
        Visualizer::makeFigure(ex2aEnv);
        Visualizer::showFigures();
    }

    /*-------- Exercise 2b --------*/
    // {
    //     amp::Problem2D ex2bEnv1 = HW2::getWorkspace1();
    //     Visualizer::makeFigure(ex2bEnv1);
    //     // MyGDAlgorithm gradAlgo{};

    //     amp::Problem2D ex2bEnv2 = HW2::getWorkspace2();
    //     Visualizer::makeFigure(ex2bEnv2);
    //     Visualizer::showFigures();
    // }
    
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

