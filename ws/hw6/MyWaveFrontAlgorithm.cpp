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
    bool waveReachStart{false};
    unsigned int currDist{};
    auto[numx0Cells, numx1Cells] = grid_cspace.size();
    DenseArray2D<unsigned int> waveGrid{numx0Cells, numx1Cells, 0};
    auto[x_min, x_max] = grid_cspace.x0Bounds();
    auto[y_min, y_max] = grid_cspace.x1Bounds();

    for (int i = 0; i < numx0Cells; i++) {
        for (int j = 0; j < numx1Cells; j++) {
            if (grid_cspace(i, j)) {
                waveGrid(i, j) = 1;
            } else {
                waveGrid(i, j) = 0;
            }
        }
    }

    search.push(grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]));
    waveGrid(search.front().first, search.front().second) = 2;
    std::pair<std::size_t, std::size_t> qInitCell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    while (!waveReachStart) {
        if (search.front() == qInitCell) {
            waveReachStart = true;
        }

        currDist = waveGrid(search.front().first, search.front().second);

        if (search.front().first > 0) {
            if (waveGrid(search.front().first - 1, search.front().second) == 0) {
                search.push(std::make_pair(search.front().first - 1, search.front().second));
                waveGrid(search.front().first - 1, search.front().second) = currDist + 1;
            }
        }

        if (search.front().first + 1 <= numx0Cells - 1) {
            if (waveGrid(search.front().first + 1, search.front().second) == 0) {
                search.push(std::make_pair(search.front().first + 1, search.front().second));
                waveGrid(search.front().first + 1, search.front().second) = currDist + 1;
            }
        }

        if (search.front().second > 0) {
            if (waveGrid(search.front().first, search.front().second - 1) == 0) {
                search.push(std::make_pair(search.front().first, search.front().second - 1));
                waveGrid(search.front().first, search.front().second - 1) = currDist + 1;
            }
        }

        if (search.front().second + 1 <= numx1Cells - 1) {
            if (waveGrid(search.front().first, search.front().second + 1) == 0) {
                search.push(std::make_pair(search.front().first, search.front().second + 1));
                waveGrid(search.front().first, search.front().second + 1) = currDist + 1;
            }
        }

        search.pop();
    }

    path.waypoints.push_back(q_init);
    auto[waveGridx, waveGridy] = grid_cspace.getCellFromPoint(path.waypoints.back()[0], path.waypoints.back()[1]);
    while ((currDist = waveGrid(waveGridx, waveGridy)) != 2) {
        if (waveGridx + 1 <= numx0Cells - 1) {
            if (waveGrid(waveGridx + 1, waveGridy) < currDist && waveGrid(waveGridx + 1, waveGridy) > 1) {
                waveGridx++;
                path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                         y_min + (cellSize * waveGridy) + (cellSize / 2)));
                continue;
            }
        }

        if (waveGridy + 1 <= numx1Cells - 1) {
            if (waveGrid(waveGridx, waveGridy + 1) < currDist && waveGrid(waveGridx, waveGridy + 1) > 1) {
                waveGridy++;
                path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                         y_min + (cellSize * waveGridy) + (cellSize / 2)));
                continue;
            }
        }

        if (waveGridx > 0) {
            if (waveGrid(waveGridx - 1, waveGridy) < currDist && waveGrid(waveGridx - 1, waveGridy) > 1) {
                waveGridx--;
                path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                         y_min + (cellSize * waveGridy) + (cellSize / 2)));
                continue;
            }
        }

        if (waveGridy > 0) {
            if (waveGrid(waveGridx, waveGridy - 1) < currDist && waveGrid(waveGridx, waveGridy - 1) > 1) {
                waveGridy--;
                path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                         y_min + (cellSize * waveGridy) + (cellSize / 2)));
                continue;
            }
        }

        break;
    }

    path.waypoints.push_back(q_goal);

    return path;
}
