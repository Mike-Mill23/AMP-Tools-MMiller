// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW8.h"

// Include the header of the shared class
#include "CollisionDetector.h"
#include "MyGridCSpace.h"
#include "KDTree.hpp"
#include "MyGoalBiasRRT.h"

// Include headers from hw ws
#include "MyCentralizedMultiAgentRRT.h"

// Include other stdlib headers
#include <iostream>


using namespace amp;

int main(int argc, char** argv) {

    /*-------- Exercise 1(b) --------*/
    {
        MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);
    }

    /*-------- Grading --------*/
    {
        
    }

    /*-------- Testing/Debugging --------*/
    {
        // MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(3);
        // MultiAgentPath2D path{};
        // for (auto agent : hw8ws1.agent_properties) {
        //     Path2D agentPath{};
        //     agentPath.waypoints.push_back(agent.q_init);
        //     agentPath.waypoints.push_back(agent.q_goal);
        //     path.agent_paths.push_back(agentPath);
        // }

        // Visualizer::makeFigure(hw8ws1, path);
        // Visualizer::showFigures();

        // HW8::check(path, hw8ws1);
    }
    
    return 0;
}
