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
#include "MyDecentralizedMultiAgentRRT.h"

// Include other stdlib headers
#include <iostream>
#include <stdio.h>


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

        HW8::check(path, hw8ws1);
        Visualizer::makeFigure(hw8ws1, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 1(c) --------*/
    {
        std::list<std::vector<double>> treeSizeDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"m = 2"};

        MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        MyCentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        for (int i = 0; i < 100; i++) {
            multiPlanner.plan(hw8ws1);
        }

        treeSizeDataSet.push_back(multiPlanner.treeSizeDataSet);
        compTimeDataSet.push_back(multiPlanner.compTimeDataSet);

        Visualizer::makeBoxPlot(treeSizeDataSet, dataSetLabels, "Tree Size for Centralized Multi Agent RRT, m = 2", "Number of Agents, m", "Tree Size [num. nodes]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Centralized Multi Agent RRT, m = 2", "Number of Agents, m", "Computation Time [ms]");
        Visualizer::showFigures();
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
                // multiPlanner.plan(hw8ws1);
                amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);
                if (HW8::check(path, hw8ws1, false) && plotPath) {
                    Visualizer::makeFigure(hw8ws1, path);
                    Visualizer::showFigures();
                    plotPath = false;
                }
            }
            // LOG("resultsFound: " << multiPlanner.resultsFound);
            // LOG("numValidSolutions: " << multiPlanner.numValidSolutions);
            treeSizeDataSet.push_back(multiPlanner.treeSizeDataSet);
            compTimeDataSet.push_back(multiPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(treeSizeDataSet, dataSetLabels, "Tree Sizes for Centralized Multi Agent RRT", "Number of Agents, m", "Tree Size [num. nodes]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Centralized Multi Agent RRT", "Number of Agents, m", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Exercise 1(e) --------*/
    {
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

            double avgTreeSize = std::accumulate(multiPlanner.treeSizeDataSet.begin(), multiPlanner.treeSizeDataSet.end(), 0.0) / multiPlanner.treeSizeDataSet.size();
            double avgCompTime = std::accumulate(multiPlanner.compTimeDataSet.begin(), multiPlanner.compTimeDataSet.end(), 0.0) / multiPlanner.compTimeDataSet.size();
            printf("m = %d:\n\tAverage Tree Size: %f\n\tAverage Comp Time: %f\n\n", benchmark, avgTreeSize, avgCompTime);
        }
    }

    /*-------- Exercise 2(b) --------*/
    {
        MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        MyDecentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);

        HW8::check(path, hw8ws1);
        Visualizer::makeFigure(hw8ws1, path);
        Visualizer::showFigures();
    }

    /*-------- Exercise 2(c) --------*/
    {
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"m = 2"};

        MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(2);

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        MyDecentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};

        for (int i = 0; i < 100; i++) {
            multiPlanner.plan(hw8ws1);
        }

        compTimeDataSet.push_back(multiPlanner.compTimeDataSet);

        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Decentralized Multi Agent RRT, m = 2", "Number of Agents, m", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Exercise 2(d) --------*/
    {
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"m = 3", "m = 4", "m = 5", "m = 6"};

        std::vector<int> benchmarks{3, 4, 5, 6};

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        bool plotPath{true};

        for (auto benchmark : benchmarks) {
            MyDecentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};
            MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(benchmark);
            plotPath = true;
            for (int i = 0; i < 100; i++) {
                // multiPlanner.plan(hw8ws1);
                amp::MultiAgentPath2D path = multiPlanner.plan(hw8ws1);
                if (HW8::check(path, hw8ws1, false) && plotPath) {
                    Visualizer::makeFigure(hw8ws1, path);
                    Visualizer::showFigures();
                    plotPath = false;
                }
            }
            // LOG("resultsFound: " << multiPlanner.resultsFound);
            // LOG("numValidSolutions: " << multiPlanner.numValidSolutions);
            compTimeDataSet.push_back(multiPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Decentralized Multi Agent RRT", "Number of Agents, m", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Exercise 2(e) --------*/
    {
        std::vector<int> benchmarks{1, 2, 3, 4, 5, 6};

        unsigned int n{7500};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};

        for (auto benchmark : benchmarks) {
            MyDecentralizedMultiAgentRRT multiPlanner{n, r, goalProb, epsilon};
            MultiAgentProblem2D hw8ws1 = HW8::getWorkspace1(benchmark);
            for (int i = 0; i < 100; i++) {
                multiPlanner.plan(hw8ws1);
            }

            double avgCompTime = std::accumulate(multiPlanner.compTimeDataSet.begin(), multiPlanner.compTimeDataSet.end(), 0.0) / multiPlanner.compTimeDataSet.size();
            printf("m = %d:\n\tAverage Comp Time: %f\n\n", benchmark, avgCompTime);
        }
    }

    /*-------- Grading --------*/
    {
        unsigned int n{250000};
        double r{0.5};
        double goalProb{0.075};
        double epsilon{0.25};

        MyCentralizedMultiAgentRRT multiCentralized{n, r, goalProb, epsilon};
        MyDecentralizedMultiAgentRRT multiDecentralized{n, r, goalProb, epsilon};

        HW8::grade(multiCentralized, multiDecentralized, "michael.miller-5@colorado.edu", argc, argv);
    }
    
    return 0;
}
