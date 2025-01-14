// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include headers from ws
#include "DecentralizedMultiAgentPRMStar.h"
#include "ParseYamlProblem.h"

// Inlude HW8 for path checking
#include "hw/HW8.h"

// Include other stdlib headers
#include <unistd.h>
#include <stdio.h>


using namespace amp;

int main(int argc, char** argv) {
    MultiAgentProblem2D problem = buildEnvironment("../../ws/final/garden_center_problem.yaml");

    unsigned int n{2000};
    unsigned int d{2};
    double mu{(23 * 18) - (2 * 40)};
    double zeta{M_PI * pow(1, 2)};
    unsigned int numTasks{8};

    double avgCompTime{0.0};
    double avgPathLength{0.0};
    double successRate{0.0};
    // int numSims{100};

    // for (int i = 0; i < numSims; i++) {
    //     LOG("Planning " << i+1 << "...");
    //     DecentralizedMultiAgentPRMStar prmStarPlanner{n, d, mu, zeta, numTasks};

    //     MultiAgentPath2D path = prmStarPlanner.plan(problem);

    //     // std::vector<std::vector<Eigen::Vector2d>> collisions{};
    //     // if (!HW8::check(path, problem, collisions)) {
    //     //     Visualizer::makeFigure(problem, path, collisions);
    //     //     Visualizer::showFigures();
    //     // }

    //     avgCompTime += prmStarPlanner.compTime;
    //     avgPathLength += prmStarPlanner.pathLength;
    //     if (HW8::check(path, problem, false)) {
    //         successRate += 1.0;
    //     }
    // }

    // avgCompTime /= numSims;
    // avgPathLength /= numSims;
    // successRate /= static_cast<double>(numSims);

    // LOG("avgCompTime: " << avgCompTime);
    // LOG("avgPathLength: " << avgPathLength);
    // LOG("successRate: " << successRate);

    DecentralizedMultiAgentPRMStar prmStarPlanner{n, d, mu, zeta, numTasks};

    MultiAgentPath2D path = prmStarPlanner.plan(problem);

    std::vector<std::vector<Eigen::Vector2d>> collisions{};
    HW8::check(path, problem, collisions);

    LOG("compTime: " << prmStarPlanner.compTime);
    LOG("pathLength: " << prmStarPlanner.pathLength);

    Visualizer::makeFigure(problem, path, collisions);
    Visualizer::showFigures();

    return 0;
}
