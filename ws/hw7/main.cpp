// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW7.h"
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
        unsigned int n{200};
        double r{1.0};
        bool pathSmoothing{false};
        MyPRM PRMPlanner{n, r, pathSmoothing};
        amp::Problem2D HW5Ws1 = HW5::getWorkspace1();
        HW5Ws1.y_min = -3;
        HW5Ws1.y_max = 3;
        amp::Path2D path = PRMPlanner.plan(HW5Ws1);
    }

    /*-------- Exercise 1(a)(ii) --------*/
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
        bool pathSmoothing{false};

        for (auto benchmark : benchmarks) {
            MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                PRMPlanner.plan(HW5Ws1);
            }

            validSolutionsDataSet.push_back(std::vector<double>{static_cast<double>(PRMPlanner.numValidSolutions)});
            pathLengthDataSet.push_back(PRMPlanner.pathLengthDataSet);
            compTimeDataSet.push_back(PRMPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(validSolutionsDataSet, dataSetLabels, "Number of Valid Solutions for Each PRM Benchmark", "Data Set n and r", "Valid Solutions");
        Visualizer::makeBoxPlot(pathLengthDataSet, dataSetLabels, "Path Lengths for Each PRM Benchmark", "Data Set n and r", "Path Lengths [units]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Each PRM Benchmark", "Data Set n and r", "Computation Time [ms]");
        Visualizer::showFigures();
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

    /*-------- Exercise 1(b)(i) Ws1 --------*/
    {
        unsigned int n{200};
        double r{2.0};
        bool pathSmoothing{false};
        MyPRM PRMPlanner{n, r, pathSmoothing};
        amp::Problem2D HW2Ws1 = HW2::getWorkspace1();
        amp::Path2D path = PRMPlanner.plan(HW2Ws1);
    }

    /*-------- Exercise 1(b)(i) Ws2 --------*/
    {
        unsigned int n{200};
        double r{2.0};
        bool pathSmoothing{false};
        MyPRM PRMPlanner{n, r, pathSmoothing};
        amp::Problem2D HW2Ws2 = HW2::getWorkspace2();
        amp::Path2D path = PRMPlanner.plan(HW2Ws2);
    }

    /*-------- Exercise 1(b)(ii) Ws1 --------*/
    {
        std::list<std::vector<double>> validSolutionsDataSet{};
        std::list<std::vector<double>> pathLengthDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"n = 200, r = 1.0", "n = 200, r = 2.0", 
                                               "n = 500, r = 1.0", "n = 500, r = 2.0", 
                                               "n = 1000, r = 1.0", "n = 1000, r = 2.0"};

        amp::Problem2D HW2Ws1 = HW2::getWorkspace1();

        std::vector<std::pair<unsigned int, double>> benchmarks{{200, 1.0}, {200, 2.0}, 
                                                                {500, 1.0}, {500, 2.0}, 
                                                                {1000, 1.0}, {1000, 2.0}};
        bool pathSmoothing{false};

        for (auto benchmark : benchmarks) {
            MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                PRMPlanner.plan(HW2Ws1);
            }

            validSolutionsDataSet.push_back(std::vector<double>{static_cast<double>(PRMPlanner.numValidSolutions)});
            pathLengthDataSet.push_back(PRMPlanner.pathLengthDataSet);
            compTimeDataSet.push_back(PRMPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(validSolutionsDataSet, dataSetLabels, "Number of Valid Solutions for Each PRM Benchmark", "Data Set n and r", "Valid Solutions");
        Visualizer::makeBoxPlot(pathLengthDataSet, dataSetLabels, "Path Lengths for Each PRM Benchmark", "Data Set n and r", "Path Lengths [units]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Each PRM Benchmark", "Data Set n and r", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Exercise 1(b)(ii) Ws2 --------*/
    {
        std::list<std::vector<double>> validSolutionsDataSet{};
        std::list<std::vector<double>> pathLengthDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"n = 200, r = 1.0", "n = 200, r = 2.0", 
                                               "n = 500, r = 1.0", "n = 500, r = 2.0", 
                                               "n = 1000, r = 1.0", "n = 1000, r = 2.0"};

        amp::Problem2D HW2Ws2 = HW2::getWorkspace2();

        std::vector<std::pair<unsigned int, double>> benchmarks{{200, 1.0}, {200, 2.0}, 
                                                                {500, 1.0}, {500, 2.0}, 
                                                                {1000, 1.0}, {1000, 2.0}};
        bool pathSmoothing{false};

        for (auto benchmark : benchmarks) {
            MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                PRMPlanner.plan(HW2Ws2);
            }

            validSolutionsDataSet.push_back(std::vector<double>{static_cast<double>(PRMPlanner.numValidSolutions)});
            pathLengthDataSet.push_back(PRMPlanner.pathLengthDataSet);
            compTimeDataSet.push_back(PRMPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(validSolutionsDataSet, dataSetLabels, "Number of Valid Solutions for Each PRM Benchmark", "Data Set n and r", "Valid Solutions");
        Visualizer::makeBoxPlot(pathLengthDataSet, dataSetLabels, "Path Lengths for Each PRM Benchmark", "Data Set n and r", "Path Lengths [units]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Each PRM Benchmark", "Data Set n and r", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Exercise 1(b)(iv) Ws1 --------*/
    {
        std::list<std::vector<double>> validSolutionsDataSet{};
        std::list<std::vector<double>> pathLengthDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"n = 200, r = 1.0", "n = 200, r = 2.0", 
                                               "n = 500, r = 1.0", "n = 500, r = 2.0", 
                                               "n = 1000, r = 1.0", "n = 1000, r = 2.0"};

        amp::Problem2D HW2Ws1 = HW2::getWorkspace1();

        std::vector<std::pair<unsigned int, double>> benchmarks{{200, 1.0}, {200, 2.0}, 
                                                                {500, 1.0}, {500, 2.0}, 
                                                                {1000, 1.0}, {1000, 2.0}};
        bool pathSmoothing{true};

        for (auto benchmark : benchmarks) {
            MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                PRMPlanner.plan(HW2Ws1);
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

    /*-------- Exercise 1(b)(iv) Ws2 --------*/
    {
        std::list<std::vector<double>> validSolutionsDataSet{};
        std::list<std::vector<double>> pathLengthDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"n = 200, r = 1.0", "n = 200, r = 2.0", 
                                               "n = 500, r = 1.0", "n = 500, r = 2.0", 
                                               "n = 1000, r = 1.0", "n = 1000, r = 2.0"};

        amp::Problem2D HW2Ws2 = HW2::getWorkspace2();

        std::vector<std::pair<unsigned int, double>> benchmarks{{200, 1.0}, {200, 2.0}, 
                                                                {500, 1.0}, {500, 2.0}, 
                                                                {1000, 1.0}, {1000, 2.0}};
        bool pathSmoothing{true};

        for (auto benchmark : benchmarks) {
            MyPRM PRMPlanner{benchmark.first, benchmark.second, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                PRMPlanner.plan(HW2Ws2);
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

    /*-------- Exercise 2(a) Ws1 --------*/
    {
        unsigned int n{5000};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};
        bool pathSmoothing{false};
        MyGoalBiasRRT RRTPlanner{n, r, goalProb, epsilon, pathSmoothing};
        amp::Problem2D HW5Ws1 = HW5::getWorkspace1();
        HW5Ws1.y_min = -3;
        HW5Ws1.y_max = 3;
        amp::Path2D path = RRTPlanner.plan(HW5Ws1);
    }

    /*-------- Exercise 2(a) Ws2 --------*/
    {
        unsigned int n{5000};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};
        bool pathSmoothing{false};
        MyGoalBiasRRT RRTPlanner{n, r, goalProb, epsilon, pathSmoothing};
        amp::Problem2D HW2Ws1 = HW2::getWorkspace1();
        amp::Path2D path = RRTPlanner.plan(HW2Ws1);
    }

    /*-------- Exercise 2(a) Ws3 --------*/
    {
        unsigned int n{5000};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};
        bool pathSmoothing{false};
        MyGoalBiasRRT RRTPlanner{n, r, goalProb, epsilon, pathSmoothing};
        amp::Problem2D HW2Ws2 = HW2::getWorkspace2();
        amp::Path2D path = RRTPlanner.plan(HW2Ws2);
    }

    /*-------- Exercise 2(b) --------*/
    {
        std::list<std::vector<double>> validSolutionsDataSet{};
        std::list<std::vector<double>> pathLengthDataSet{};
        std::list<std::vector<double>> compTimeDataSet{};
        std::vector<std::string> dataSetLabels{"Homework 5.2, Workspace 1", 
                                               "Homework 2.2, Workspace 1", 
                                               "Homework 2.2, Workspace 2"};


        unsigned int n{5000};
        double r{0.5};
        double goalProb{0.05};
        double epsilon{0.25};
        bool pathSmoothing{false};

        amp::Problem2D HW5Ws1 = HW5::getWorkspace1();
        amp::Problem2D HW2Ws1 = HW2::getWorkspace1();
        amp::Problem2D HW2Ws2 = HW2::getWorkspace2();

        std::vector<amp::Problem2D> benchmarks{HW5Ws1, HW2Ws1, HW2Ws2};

        for (auto benchmark : benchmarks) {
            MyGoalBiasRRT RRTPlanner{n, r, goalProb, epsilon, pathSmoothing};
            for (int i = 0; i < 100; i++) {
                RRTPlanner.plan(benchmark);
            }

            validSolutionsDataSet.push_back(std::vector<double>{static_cast<double>(RRTPlanner.numValidSolutions)});
            pathLengthDataSet.push_back(RRTPlanner.pathLengthDataSet);
            compTimeDataSet.push_back(RRTPlanner.compTimeDataSet);
        }

        Visualizer::makeBoxPlot(validSolutionsDataSet, dataSetLabels, "Number of Valid Solutions for Each GoalBiasRRT Benchmark", "Homework, Workspace", "Valid Solutions");
        Visualizer::makeBoxPlot(pathLengthDataSet, dataSetLabels, "Path Lengths for Each GoalBiasRRT Benchmark", "Homework, Workspace", "Path Lengths [units]");
        Visualizer::makeBoxPlot(compTimeDataSet, dataSetLabels, "Computation Times for Each GoalBiasRRT Benchmark", "Homework, Workspace", "Computation Time [ms]");
        Visualizer::showFigures();
    }

    /*-------- Grading --------*/
    {
        unsigned int nPRM{500};
        double rPRM{2.0};
        bool pathSmoothing{true};
        MyPRM PRMPlanner{nPRM, rPRM, pathSmoothing};

        unsigned int nRRT{5000};
        double rRRT{0.5};
        double goalProb{0.05};
        double epsilon{0.25};
        MyGoalBiasRRT RRTPlanner{nRRT, rRRT, goalProb, epsilon, pathSmoothing};

        amp::HW7::grade(PRMPlanner, RRTPlanner, "michael.miller-5@colorado.edu", argc, argv);
    }
    
    return 0;
}
