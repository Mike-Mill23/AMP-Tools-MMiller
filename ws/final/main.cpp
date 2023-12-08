// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the header of the shared class

// Include headers from hw ws
#include "DecentralizedMultiAgentPRMStar.h"
#include "ParseYamlProblem.h"

// Include other stdlib headers
#include <unistd.h>
#include <stdio.h>


using namespace amp;

int main(int argc, char** argv) {
    MultiAgentProblem2D problem = buildEnvironment("../../ws/final/garden_center_problem.yaml");

    unsigned int n{5000};
    unsigned int d{2};
    double mu{(21 * 16) - (2 * 40)};
    double zeta{M_PI * pow(1, 2)};
    DecentralizedMultiAgentPRMStar prmStarPlanner{n, d, mu, zeta};

    MultiAgentPath2D path = prmStarPlanner.plan(problem);

    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();

    return 0;
}
