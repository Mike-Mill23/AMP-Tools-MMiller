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
        // MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);

        // unsigned int n{7500};
        // double r{0.5};
        // double goalProb{0.05};
        // double epsilon{0.25};

        // MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        // amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);

        // HW8::check(path, hw8ws1);
        // Visualizer::makeFigure(hw8ws1, path);
        // Visualizer::showFigures();
    }

    /*-------- Exercise 1(c) --------*/
    {
        // std::list<std::vector<double>> treeSizeDataSet{};
        // std::list<std::vector<double>> compTimeDataSet{};
        // std::vector<std::string> dataSetLabels{"m = 2"};

        // MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);

        // unsigned int n{7500};
        // double r{0.5};
        // double goalProb{0.05};
        // double epsilon{0.25};

        // MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        // for (int i = 0; i < 100; i++) {
        //     multiPlanner.plan(hw8ws1);
        // }

        // treeSizeDataSet.push_back(multiPlanner.treeSizeDataSet);
        // compTimeDataSet.push_back(multiPlanner.compTimeDataSet);

        // Visualizer::makeBoxPlot(treeSizeDataSet, dataSetLabels, "Tree Size for Centralized Multi Agent RRT, m = 2", "Number of Agents, m", "Tree Size [num. nodes]");
        // Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Centralized Multi Agent RRT, m = 2", "Number of Agents, m", "Computation Time [ms]");
        // Visualizer::showFigures();
    }

    /*-------- Exercise 1(d) --------*/
    {
        std::list<std::vector<double>> treeSizeDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"m = 3", "m = 4", "m = 5", "m = 6"};

        std::vector<int> benchmarks{3, 4, 5, 6};

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        bool plotPath{true};

        for (auto benchmark : benchmarks) {
            MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};
            MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(benchmark);
            plotPath = true;
            for (int i = 0; i < 100; i++) {
                amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);
                if (HW8::check(path, hw8ws1, false) && plotPath) {
                    Visualizer::makeFigure(hw8ws1, path);
                    Visualizer::showFigures();
                    plotPath = false;
                }
            }

            treeSizeDataSet.push_back(multiPlanner.treeSizeDataSet);
            compTimeDataSet.push_back(multiPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(treeSizeDataSet, dataSetLabels, "Tree Sizes for Centralized Multi Agent RRT", "Number of Agents, m", "Tree Size [num. nodes]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Centralized Multi Agent RRT", "Number of Agents, m", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Exercise 1(e) --------*/
    {
        std::vector<double> avgTreeSizes{};
        std::vector<double> avgCompTimes{};
        std::vector<std::string> dataSetLabels{"m = 1", "m = 2", "m = 3", "m = 4", "m = 5", "m = 6"};

        std::vector<int> benchmarks{1, 2, 3, 4, 5, 6};

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        for (auto benchmark : benchmarks) {
            MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};
            MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(benchmark);
            for (int i = 0; i < 100; i++) {
                multiPlanner.plan(hw8ws1);
            }

            treeSizeDataSet.push_back(multiPlanner.treeSizeDataSet);
            compTimeDataSet.push_back(multiPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(treeSizeDataSet, dataSetLabels, "Tree Sizes for Centralized Multi Agent RRT", "Number of Agents, m", "Tree Size [num. nodes]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Centralized Multi Agent RRT", "Number of Agents, m", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Grading --------*/
    {
        
    }

    /*-------- Testing/Debugging --------*/
    {
        // Min. Path Check
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

        // Valid Solution Benchmark
        // MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);
        // std::vector<std::vector<Eigen::Vector2d>> collisions{};

        // unsigned int n{7500};
        // double r{0.5};
        // double goalProb{0.05};
        // double epsilon{0.25};

        // int numValidSolutions{0};

        // MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        // for (int i = 0; i < 100; i++) {
        //     amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);
        //     if (HW8::check(path, hw8ws1, collisions)) {
        //         numValidSolutions++;
        //     } else {
        //         LOG("collisions.size(): " << collisions.size());
        //         for (int j = 0; j < collisions.size(); j++) {
        //             for (int k = 0; k < collisions[j].size(); k++) {
        //                 LOG("-----------------\n" << collisions[j][k]);
        //             }
        //         }
        //         amp::MultiAgentPath2D path1 = path;
        //         path1.agent_paths[1].waypoints.erase(path1.agent_paths[1].waypoints.begin() + 1, path1.agent_paths[1].waypoints.end() - 1);
        //         amp::MultiAgentPath2D path2 = path;
        //         path2.agent_paths[0].waypoints.erase(path2.agent_paths[0].waypoints.begin() + 1, path2.agent_paths[0].waypoints.end() - 1);
        //         Visualizer::makeFigure(hw8ws1, path);
        //         Visualizer::makeFigure(hw8ws1, path1);
        //         Visualizer::makeFigure(hw8ws1, path2);
        //         Visualizer::showFigures();
        //     }
        // }
        // LOG("num valid solutions: " << numValidSolutions << "/100");
    }
    
    return 0;
}
