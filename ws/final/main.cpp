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
    double mu{(21 * 16) - (2 * 40)};
    double zeta{M_PI * pow(1, 2)};

    // for (int i = 0; i < 100; i++) {
    //     LOG("Planning " << i+1 << "...");
    //     DecentralizedMultiAgentPRMStar prmStarPlanner{n, d, mu, zeta};

    //     MultiAgentPath2D path = prmStarPlanner.plan(problem);

    //     std::vector<std::vector<Eigen::Vector2d>> collisions{};
    //     if (!HW8::check(path, problem, collisions)) {
    //         Visualizer::makeFigure(problem, path, collisions);
    //         Visualizer::showFigures();
    //     }
    // }

    DecentralizedMultiAgentPRMStar prmStarPlanner{n, d, mu, zeta};

    MultiAgentPath2D path = prmStarPlanner.plan(problem);

    std::vector<std::vector<Eigen::Vector2d>> collisions{};
    HW8::check(path, problem, collisions);

    // for (auto path2d : path.agent_paths) {
    //     path2d.print();
    // }

    Visualizer::makeFigure(problem, path, collisions);
    Visualizer::showFigures();

    return 0;
}
