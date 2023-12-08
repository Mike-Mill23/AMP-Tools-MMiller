#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "CollisionDetector.h"

#include <random>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>

namespace amp {
    class MyAStar : public AStar {
        public:
            MyAStar() = default;
            ~MyAStar() = default;

            /// @brief Find the shortest path from an init node to a goal node on a graph, while using a heuristic.
            /// @param problem Search problem containing init/goal nodes and graph
            /// @param heuristic Heuristic function that maps each node to a "cost-to-go"
            /// @return The optimal node path and path cost (if the heuristic is admissible)
            GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic);

        private:
            struct nodeList {
                amp::Node currNode{};
                amp::Node parentNode{};
                double cost{};
                double distance{};

                nodeList(amp::Node currNode = 0, amp::Node parentNode = 0, double cost = 0.0, double distance = 0.0) : currNode(currNode), parentNode(parentNode), cost(cost), distance(distance) {}
                bool operator==(nodeList n) { return (this->currNode == n.currNode) && (this->parentNode == n.parentNode) && (this->cost == n.cost) && (this->distance == n.distance); }
            };
    };

    class PRMSearchHeuristic : public SearchHeuristic {
        public:
            PRMSearchHeuristic(const amp::ShortestPathProblem& problem, std::unique_ptr<std::vector<std::vector<double>>>& sampledPoints);
            ~PRMSearchHeuristic() = default;

            /// @brief Get the heuristic value stored in `heuristicValues`. 
            /// @param node Node to get the heuristic value h(node) for. 
            /// @return Heuristic value
            double operator()(amp::Node node) const { return heuristicValues.at(node); }

        private:
            std::map<amp::Node, double> heuristicValues;
    };

    class RRTSearchHeuristic : public SearchHeuristic {
        public:
            RRTSearchHeuristic(const amp::ShortestPathProblem& problem, std::unique_ptr<std::vector<std::vector<double>>>& sampledPoints);
            ~RRTSearchHeuristic() = default;

            /// @brief Get the heuristic value stored in `heuristicValues`. 
            /// @param node Node to get the heuristic value h(node) for. 
            /// @return Heuristic value
            double operator()(amp::Node node) const { return heuristicValues.at(node); }

        private:
            std::map<amp::Node, double> heuristicValues;
    };

    class CentralizedRRTSearchHeuristic : public SearchHeuristic {
        public:
            CentralizedRRTSearchHeuristic(const amp::ShortestPathProblem& problem, std::unique_ptr<std::vector<std::vector<double>>>& sampledPoints);
            ~CentralizedRRTSearchHeuristic() = default;

            /// @brief Get the heuristic value stored in `heuristicValues`. 
            /// @param node Node to get the heuristic value h(node) for. 
            /// @return Heuristic value
            double operator()(amp::Node node) const { return heuristicValues.at(node); }

        private:
            std::map<amp::Node, double> heuristicValues;
    };

    class PRMStarSearchHeuristic : public SearchHeuristic {
        public:
            PRMStarSearchHeuristic(const amp::ShortestPathProblem& problem, std::unique_ptr<std::vector<Eigen::Vector2d>>& sampledPoints);
            ~PRMStarSearchHeuristic() = default;

            /// @brief Get the heuristic value stored in `heuristicValues`. 
            /// @param node Node to get the heuristic value h(node) for. 
            /// @return Heuristic value
            double operator()(amp::Node node) const { return heuristicValues.at(node); }

        private:
            std::map<amp::Node, double> heuristicValues;
    };
}
