# include "MyWaveFrontAlgorithm.h"

using namespace amp;

std::unique_ptr<amp::GridCSpace2D> MyPointWaveFrontAlgorithm::constructDiscretizedWorkspace(const amp::Environment2D& environment) {
    const double cellSize{0.25};
    bool isCollision{false};
    const unsigned int numx0Cells = (environment.x_max - environment.x_min) / cellSize;
    const unsigned int numx1Cells = (environment.y_max - environment.y_min) / cellSize;
    std::unique_ptr<amp::MyGridCSpace2D> cSpace = std::make_unique<amp::MyGridCSpace2D>(numx0Cells, numx1Cells, environment.x_min, environment.x_max, environment.y_min, environment.y_max);

    for (int i = 0; i < numx0Cells; i++) {
        double x0 = environment.x_min + (cellSize * i) + (cellSize / 2);
        for (int j = 0; j < numx1Cells; j++) {
            double x1 = environment.y_min + (cellSize * j) + (cellSize / 2);

            for (int k = 0; k < environment.obstacles.size(); k++) {
                if (isCollision = collisionPointPolygon(Eigen::Vector2d(x0, x1), environment.obstacles[k])) {
                    break;
                }
            }

            if (isCollision) {
                (*cSpace)(i, j) = true;
            } else {
                (*cSpace)(i, j) = false;
            }
        }
    }

    return cSpace;
}

amp::Path2D MyPointWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    const double cellSize{0.25};
    amp::Path2D path{};
    std::queue<std::pair<std::size_t, std::size_t>> search{};
    std::vector<std::pair<std::size_t, std::size_t>> neighbors{};
    bool waveReachStart{false};
    unsigned int currDist{};
    auto[numx0Cells, numx1Cells] = grid_cspace.size();
    DenseArray2D<unsigned int> waveGrid{numx0Cells, numx1Cells, 0u};
    auto[x_min, x_max] = grid_cspace.x0Bounds();
    auto[y_min, y_max] = grid_cspace.x1Bounds();

    std::pair<std::size_t, std::size_t> qInitCell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    std::pair<std::size_t, std::size_t> qGoalCell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

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
    path.waypoints.push_back(q_init);
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

    path.waypoints.push_back(q_goal);
    // Visualizer::makeFigure(grid_cspace, path);
    // Visualizer::showFigures();

    return path;
}

std::vector<std::pair<std::size_t, std::size_t>> MyPointWaveFrontAlgorithm::getNeighbors(const std::pair<std::size_t, std::size_t>& cell, const std::size_t& numx0Cells, const std::size_t& numx1Cells) {
    std::vector<std::pair<std::size_t, std::size_t>> neighbors{};

    // Right neighbor
    if (cell.first + 1 < numx0Cells) {
        neighbors.push_back(std::make_pair(cell.first + 1, cell.second));
    }

    // Up neighbor
    if (cell.second + 1 < numx1Cells) {
        neighbors.push_back(std::make_pair(cell.first, cell.second + 1));
    }

    // Left neighbor
    if (cell.first > 0) {
        neighbors.push_back(std::make_pair(cell.first - 1, cell.second));
    }

    // Down neighbor
    if (cell.second > 0) {
        neighbors.push_back(std::make_pair(cell.first, cell.second - 1));
    }

    return neighbors;
}
