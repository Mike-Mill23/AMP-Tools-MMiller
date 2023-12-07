// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the header of the shared class

// Include headers from hw ws
#include "ParseYamlProblem.h"

// Include other stdlib headers
#include <unistd.h>
#include <stdio.h>


using namespace amp;

int main(int argc, char** argv) {
    MultiAgentProblem2D problem = buildEnvironment("../../ws/final/garden_center_problem.yaml");
    MultiAgentPath2D path{};

    path.agent_paths.push_back(Path2D());
    path.agent_paths[0].waypoints.push_back(problem.agent_properties[0].q_init);
    path.agent_paths[0].waypoints.push_back(problem.agent_properties[0].q_goal);

    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();

    return 0;
}
