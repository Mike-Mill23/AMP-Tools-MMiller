# include "MyManipulatorWaveFrontAlgorithm.h"

using namespace amp;

amp::Path2D MyManipulatorWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    const double cellSize{0.01};
    amp::Path2D path{};
    std::queue<std::pair<std::size_t, std::size_t>> search{};
    std::vector<std::pair<std::size_t, std::size_t>> neighbors{};
    bool waveReachStart{false};
    unsigned int currDist{};
    auto[numx0Cells, numx1Cells] = grid_cspace.size();
    DenseArray2D<unsigned int> waveGrid{numx0Cells, numx1Cells, 0u};
    auto[x_min, x_max] = grid_cspace.x0Bounds();
    auto[y_min, y_max] = grid_cspace.x1Bounds();
    Eigen::Vector2d q_initMod = q_init;
    Eigen::Vector2d q_goalMod = q_goal;

    if (q_initMod[0] < 0) {
        q_initMod[0] += 2 * M_PI;
    }
    if (q_initMod[1] < 0) {
        q_initMod[1] += 2 * M_PI;
    }

    if (q_goalMod[0] < 0) {
        q_goalMod[0] += 2 * M_PI;
    }
    if (q_goalMod[1] < 0) {
        q_goalMod[1] += 2 * M_PI;
    }

    std::pair<std::size_t, std::size_t> qInitCell = getInitCell(grid_cspace, q_initMod, numx0Cells, numx1Cells);
    std::pair<std::size_t, std::size_t> qGoalCell = getGoalCell(grid_cspace, q_goalMod, numx0Cells, numx1Cells);

    for (std::size_t i = 0; i < numx0Cells; i++) {
        for (std::size_t j = 0; j < numx1Cells; j++) {
            if (grid_cspace(i, j)) {
                waveGrid(i, j) = 1;
                neighbors = getNeighbors(std::make_pair(i, j), numx0Cells, numx1Cells);
                for (auto cell : neighbors) {
                    waveGrid(cell.first, cell.second) = 1;
                }
            }
        }
    }
    waveGrid(qInitCell.first, qInitCell.second) = 0;
    waveGrid(qGoalCell.first, qGoalCell.second) = 0;

    search.push(qGoalCell);
    waveGrid(search.front().first, search.front().second) = 2;
    while (!waveReachStart && search.size() > 0) {
        if (search.front() == qInitCell) {
            waveReachStart = true;
        }

        currDist = waveGrid(search.front().first, search.front().second);
        neighbors = getNeighbors(search.front(), numx0Cells, numx1Cells);

        for (auto cell : neighbors) {
            if (waveGrid(cell.first, cell.second) == 0) {
                search.push(cell);
                waveGrid(cell.first, cell.second) = currDist + 1;
            }
        }

        search.pop();
    }

    bool stepped{false};
    path.waypoints.push_back(q_initMod);
    if (waveReachStart) {
        auto[waveGridx, waveGridy] = grid_cspace.getCellFromPoint(path.waypoints.back()[0], path.waypoints.back()[1]);
        
        while ((currDist = waveGrid(waveGridx, waveGridy)) != 2) {
            neighbors = getNeighbors(std::make_pair(waveGridx, waveGridy), numx0Cells, numx1Cells);

            stepped = false;
            for (auto cell : neighbors) {
                if (waveGrid(cell.first, cell.second) < currDist && waveGrid(cell.first, cell.second) > 1) {
                    stepped = true;
                    waveGridx = cell.first;
                    waveGridy = cell.second;
                    path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                             y_min + (cellSize * waveGridy) + (cellSize / 2)));
                    break;
                }
            }

            if (stepped) {
                continue;
            } else {
                break;
            }
        }
    }

    path.waypoints.push_back(q_goalMod);
    // Uncomment to see CSpace path
    // path.print();
    Visualizer::makeFigure(grid_cspace, path);
    Visualizer::showFigures();
    unwrapPath(path, Eigen::Vector2d(x_min, y_min), Eigen::Vector2d(x_max, y_max));

    return path;
}

std::vector<std::pair<std::size_t, std::size_t>> MyManipulatorWaveFrontAlgorithm::getNeighbors(const std::pair<std::size_t, std::size_t>& cell, const std::size_t& numx0Cells, const std::size_t& numx1Cells) {
    std::vector<std::pair<std::size_t, std::size_t>> neighbors{};

    // Right neighbor
    neighbors.push_back(std::make_pair((cell.first + 1) % numx0Cells, cell.second));

    // Up neighbor
    neighbors.push_back(std::make_pair(cell.first, (cell.second + 1) % numx1Cells));

    // Left neighbor
    neighbors.push_back(std::make_pair((cell.first - 1 + numx0Cells) % numx0Cells, cell.second));

    // Down neighbor
    neighbors.push_back(std::make_pair(cell.first, (cell.second - 1 + numx1Cells) % numx1Cells));

    return neighbors;
}

std::pair<std::size_t, std::size_t> MyManipulatorWaveFrontAlgorithm::getInitCell(const amp::GridCSpace2D& grid_cspace, Eigen::Vector2d& q_init, const std::size_t& numx0Cells, const std::size_t& numx1Cells) {
    double initAngle = atan2(wfProblem.q_init[1], wfProblem.q_init[0]);
    double invConfig1 = -q_init[0] + (2 * M_PI) + (2 * initAngle);
    if (invConfig1 < 0.0) {
        invConfig1 += 2 * M_PI;
    } else if (invConfig1 > 2 * M_PI) {
        invConfig1 -= 2 * M_PI;
    }
    std::pair<std::size_t, std::size_t> qInit1Cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair<std::size_t, std::size_t> qInit2Cell = grid_cspace.getCellFromPoint(invConfig1, -q_init[1] + (2 * M_PI));

    bool init1 = grid_cspace(qInit1Cell.first, qInit1Cell.second);
    bool init2 = grid_cspace(qInit2Cell.first, qInit2Cell.second);

    if (!init1 && !init2) {
        bool init1HasNeighbors{false};
        bool init2HasNeighbors{false};

        std::vector<std::pair<std::size_t, std::size_t>> init1Neighbors = getNeighbors(qInit1Cell, numx0Cells, numx1Cells);
        std::vector<std::pair<std::size_t, std::size_t>> init2Neighbors = getNeighbors(qInit2Cell, numx0Cells, numx1Cells);

        for (auto neighbor : init1Neighbors) {
            if (grid_cspace(neighbor.first, neighbor.second)) {
                init1HasNeighbors = true;
                break;
            }
        }

        for (auto neighbor : init2Neighbors) {
            if (grid_cspace(neighbor.first, neighbor.second)) {
                init2HasNeighbors = true;
                break;
            }
        }

        if ((init1HasNeighbors == init2HasNeighbors) || init2HasNeighbors) {
            return qInit1Cell;
        } else {
            q_init = Eigen::Vector2d(invConfig1, -q_init[1] + (2 * M_PI));
            return qInit2Cell;
        }
    } else {
        if (init1) {
            q_init = Eigen::Vector2d(invConfig1, -q_init[1] + (2 * M_PI));
            return qInit2Cell;
        } else {
            return qInit1Cell;
        }
    }
}

std::pair<std::size_t, std::size_t> MyManipulatorWaveFrontAlgorithm::getGoalCell(const amp::GridCSpace2D& grid_cspace, Eigen::Vector2d& q_goal, const std::size_t& numx0Cells, const std::size_t& numx1Cells) {
    double goalAngle = atan2(wfProblem.q_goal[1], wfProblem.q_goal[0]);
    double invConfig1 = -q_goal[0] + (2 * M_PI) + (2 * goalAngle);
    if (invConfig1 < 0.0) {
        invConfig1 += 2 * M_PI;
    } else if (invConfig1 > 2 * M_PI) {
        invConfig1 -= 2 * M_PI;
    }
    std::pair<std::size_t, std::size_t> qGoal1Cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);
    std::pair<std::size_t, std::size_t> qGoal2Cell = grid_cspace.getCellFromPoint(invConfig1, -q_goal[1] + (2 * M_PI));

    bool goal1 = grid_cspace(qGoal1Cell.first, qGoal1Cell.second);
    bool goal2 = grid_cspace(qGoal2Cell.first, qGoal2Cell.second);

    if (!goal1 && !goal2) {
        bool goal1HasNeighbors{false};
        bool goal2HasNeighbors{false};

        std::vector<std::pair<std::size_t, std::size_t>> goal1Neighbors = getNeighbors(qGoal1Cell, numx0Cells, numx1Cells);
        std::vector<std::pair<std::size_t, std::size_t>> goal2Neighbors = getNeighbors(qGoal2Cell, numx0Cells, numx1Cells);

        for (auto neighbor : goal1Neighbors) {
            if (grid_cspace(neighbor.first, neighbor.second)) {
                goal1HasNeighbors = true;
                break;
            }
        }

        for (auto neighbor : goal2Neighbors) {
            if (grid_cspace(neighbor.first, neighbor.second)) {
                goal2HasNeighbors = true;
                break;
            }
        }

        if ((goal1HasNeighbors == goal2HasNeighbors) || goal2HasNeighbors) {
            return qGoal1Cell;
        } else {
            q_goal = Eigen::Vector2d(invConfig1, -q_goal[1] + (2 * M_PI));
            return qGoal2Cell;
        }
    } else {
        if (goal1) {
            q_goal = Eigen::Vector2d(invConfig1, -q_goal[1] + (2 * M_PI));
            return qGoal2Cell;
        } else {
            return qGoal1Cell;
        }
    }
}
