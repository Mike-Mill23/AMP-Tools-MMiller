// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
// #include "hw/HW7.h"
#include "hw/HW2.h"
#include "hw/HW5.h"

// Include the header of the shared class
#include "CollisionDetector.h"
#include "MyGridCSpace.h"
#include "MyLinkManipulator.h"
#include "KDTree.hpp"

// Include headers from hw ws
#include "MyGoalBiasRRT.h"
#include "MyPRM.h"

// Include other stdlib headers
#include <iostream>


using namespace amp;

int main(int argc, char** argv) {

    /*-------- Exercise 1(a)(i) --------*/
    {
        // unsigned int n{200};
        // double r{1.0};
        // bool pathSmoothing{false};
        // MyPRM PRMPlanner{n, r, pathSmoothing};
        // amp::Problem2D HW5Ws1 = HW5::getWorkspace1();
        // HW5Ws1.y_min = -3;
        // HW5Ws1.y_max = 3;
        // amp::Path2D path = PRMPlanner.plan(HW5Ws1);
    }

    /*-------- Exercise 1(a)(ii) --------*/
    {
        // std::list<std::vector<double>> validSolutionsDataSet{};
        // std::list<std::vector<double>> pathLengthDataSet{};
        // std::list<std::vector<double>> compTimeDataSet{};
        // std::vector<std::string> dataSetLabels{"n = 200, r = 0.5", "n = 200, r = 1.0", "n = 200, r = 1.5", "n = 200, r = 2.0", 
        //                                        "n = 500, r = 0.5", "n = 500, r = 1.0", "n = 500, r = 1.5", "n = 500, r = 2.0"};

        // amp::Problem2D HW5Ws1 = HW5::getWorkspace1();
        // HW5Ws1.y_min = -3;
        // HW5Ws1.y_max = 3;

        // std::vector<std::pair<unsigned int, double>> benchmarks{{200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0}, 
        //                                                         {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}};
        // bool pathSmoothing{false};

        // for (auto benchmark : benchmarks) {
        //     MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
        //     for (int i = 0; i < 100; i++) {
        //         PRMPlanner.plan(HW5Ws1);
        //     }

        //     validSolutionsDataSet.push_back(std::vector<double>{static_cast<double>(PRMPlanner.numValidSolutions)});
        //     pathLengthDataSet.push_back(PRMPlanner.pathLengthDataSet);
        //     compTimeDataSet.push_back(PRMPlanner.compTimeDataSet);
        // }

        // Visualizer::makeBoxPlot(validSolutionsDataSet, dataSetLabels, "Number of Valid Solutions for Each PRM Benchmark", "Data Set n and r", "Valid Solutions");
        // Visualizer::makeBoxPlot(pathLengthDataSet, dataSetLabels, "Path Lengths for Each PRM Benchmark", "Data Set n and r", "Path Lengths [units]");
        // Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Each PRM Benchmark", "Data Set n and r", "Computation Time [ms]");
        // Visualizer::showFigures();
    }

    /*-------- Exercise 1(a)(iv) --------*/
    {
        std::list<std::vector<double>> validSolutionsDataSet{};
        std::list<std::vector<double>> pathLengthDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"n = 200, r = 0.5", "n = 200, r = 1.0", "n = 200, r = 1.5", "n = 200, r = 2.0", 
                                               "n = 500, r = 0.5", "n = 500, r = 1.0", "n = 500, r = 1.5", "n = 500, r = 2.0"};

        amp::Problem2D HW5Ws1 = HW5::getWorkspace1();
        HW5Ws1.y_min = -3;
        HW5Ws1.y_max = 3;

        std::vector<std::pair<unsigned int, double>> benchmarks{{200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0}, 
                                                                {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}};
        bool pathSmoothing{true};

        for (auto benchmark : benchmarks) {
            MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                PRMPlanner.plan(HW5Ws1);
            }

            validSolutionsDataSet.push_back(std::vector<double>{static_cast<double>(PRMPlanner.numValidSolutions)});
            pathLengthDataSet.push_back(PRMPlanner.pathLengthDataSet);
            compTimeDataSet.push_back(PRMPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(validSolutionsDataSet, dataSetLabels, "Number of Valid Solutions for Each PRM Benchmark, Post-Process Smoothed Path", "Data Set n and r", "Valid Solutions");
        Visualizer::makeBoxPlot(pathLengthDataSet, dataSetLabels, "Path Lengths for Each PRM Benchmark, Post-Process Smoothed Path", "Data Set n and r", "Path Lengths [units]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Each PRM Benchmark, Post-Process Smoothed Path", "Data Set n and r", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Grading --------*/
    {
        
    }
    
    return 0;
}
