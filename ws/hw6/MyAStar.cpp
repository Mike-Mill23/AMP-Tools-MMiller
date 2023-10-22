# include "MyAStar.h"

using namespace amp;


amp::AStar::GraphSearchResult MyAStar::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::vector<nodeList> openNodes{};
    std::vector<nodeList> closedNodes{};
    GraphSearchResult result{};
    nodeList searchNode{};
    bool isClosed{false};
    bool isOpened{false};
    bool goalFound{false};
    int iterCount{1};

    openNodes.push_back(nodeList(problem.init_node, problem.init_node, heuristic(problem.init_node), 0.0));

    while (openNodes.size() > 0) {
        searchNode = openNodes.front();
        closedNodes.push_back(searchNode);
        std::vector<Node> nodeChildren = problem.graph->children(searchNode.currNode);
        std::vector<double> childEdges = problem.graph->outgoingEdges(searchNode.currNode);
        openNodes.erase(openNodes.begin());

        for (int i = 0; i < nodeChildren.size(); i++) {
            isClosed = false;
            for (int j = 0; j < closedNodes.size(); j++) {
                if (nodeChildren[i] == closedNodes[j].currNode) {
                    isClosed = true;
                    double closedDist = searchNode.distance + childEdges[i];
                    double closedCost = heuristic(closedNodes[j].currNode) + closedDist;
                    if (closedCost < closedNodes[j].cost) {
                        closedNodes[j].cost = closedCost;
                        closedNodes[j].distance = closedDist;
                        closedNodes[j].parentNode = searchNode.currNode;
                    }
                    break;
                }
            }

            if (!isClosed) {
                double childDist = searchNode.distance + childEdges[i];
                double childCost = heuristic(nodeChildren[i]) + childDist;

                isOpened = false;
                for (int j = 0; j < openNodes.size(); j++) {
                    if (nodeChildren[i] == openNodes[j].currNode) {
                        isOpened = true;
                        if (childCost < openNodes[j].cost) {
                            openNodes[j].cost = childCost;
                            openNodes[j].distance = childDist;
                            openNodes[j].parentNode = searchNode.currNode;
                        }
                        break;
                    }
                }

                if (!isOpened) {
                    openNodes.push_back(nodeList(nodeChildren[i], searchNode.currNode, childCost, childDist));
                }
            }
        }

        std::sort(openNodes.begin(), openNodes.end(), [](nodeList n1, nodeList n2) { return n1.cost < n2.cost; });

        if (openNodes.front().currNode == problem.goal_node) {
            goalFound = true;
            break;
        }
        iterCount++;
    }

    if (goalFound) {
        result.success = goalFound;
        result.path_cost = openNodes.front().distance;

        result.node_path.push_front(openNodes.front().currNode);
        result.node_path.push_front(openNodes.front().parentNode);

        Node backTrack = openNodes.front().parentNode;
        while (result.node_path.front() != problem.init_node) {
            for (int i = 0; i < closedNodes.size(); i++) {
                if (closedNodes[i].currNode == backTrack) {
                    backTrack = closedNodes[i].parentNode;
                }
            }
            result.node_path.push_front(backTrack);
        }
    } else {
        result.success = goalFound;
        result.path_cost = -1.0;
    }

    // LOG("Number of Iterations: " << iterCount);

    return result;
}

// MySearchHeuristic::MySearchHeuristic(const amp::ShortestPathProblem& problem) {

// }
