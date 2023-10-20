# include "MyManipulatorWaveFrontAlgorithm.h"

using namespace amp;

amp::Path2D MyManipulatorWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    const double cellSize{0.01};
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
    std::size_t searchIndex{};
    while (!waveReachStart) {
        if (search.front() == qInitCell) {
            waveReachStart = true;
        }

        currDist = waveGrid(search.front().first, search.front().second);

        if (search.front().first > 0) {
            searchIndex = search.front().first - 1;
        } else {
            searchIndex = numx0Cells - 1;
        }
        if (waveGrid(searchIndex, search.front().second) == 0) {
            search.push(std::make_pair(searchIndex, search.front().second));
            waveGrid(searchIndex, search.front().second) = currDist + 1;
        }

        if (search.front().first < numx0Cells - 1) {
            searchIndex = search.front().first + 1;
        } else {
            searchIndex = 0;
        }
        if (waveGrid(searchIndex, search.front().second) == 0) {
            search.push(std::make_pair(searchIndex, search.front().second));
            waveGrid(searchIndex, search.front().second) = currDist + 1;
        }

        if (search.front().second > 0) {
            searchIndex = search.front().second - 1;
        } else {
            searchIndex = numx1Cells - 1;
        }
        if (waveGrid(search.front().first, searchIndex) == 0) {
            search.push(std::make_pair(search.front().first, searchIndex));
            waveGrid(search.front().first, searchIndex) = currDist + 1;
        }

        if (search.front().second < numx1Cells - 1) {
            searchIndex = search.front().second + 1;
        } else {
            searchIndex = 0;
        }
        if (waveGrid(search.front().first, searchIndex) == 0) {
            search.push(std::make_pair(search.front().first, searchIndex));
            waveGrid(search.front().first, searchIndex) = currDist + 1;
        }

        search.pop();
    }

    path.waypoints.push_back(q_init);
    auto[waveGridx, waveGridy] = grid_cspace.getCellFromPoint(path.waypoints.back()[0], path.waypoints.back()[1]);
    while ((currDist = waveGrid(waveGridx, waveGridy)) != 2) {
        if (waveGridx < numx0Cells - 1) {
            searchIndex = waveGridx + 1;
        } else {
            searchIndex = 0;
        }
        if (waveGrid(searchIndex, waveGridy) < currDist && waveGrid(searchIndex, waveGridy) > 1) {
            waveGridx = searchIndex;
            path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                     y_min + (cellSize * waveGridy) + (cellSize / 2)));
            continue;
        }

        if (waveGridy < numx1Cells - 1) {
            searchIndex = waveGridy + 1;
        } else {
            searchIndex = 0;
        }
        if (waveGrid(waveGridx, searchIndex) < currDist && waveGrid(waveGridx, searchIndex) > 1) {
            waveGridy = searchIndex;
            path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                     y_min + (cellSize * waveGridy) + (cellSize / 2)));
            continue;
        }

        if (waveGridx > 0) {
            searchIndex = waveGridx - 1;
        } else {
            searchIndex = numx0Cells - 1;
        }
        if (waveGrid(searchIndex, waveGridy) < currDist && waveGrid(searchIndex, waveGridy) > 1) {
            waveGridx = searchIndex;
            path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                     y_min + (cellSize * waveGridy) + (cellSize / 2)));
            continue;
        }

        if (waveGridy > 0) {
            searchIndex = waveGridy - 1;
        } else {
            searchIndex = numx1Cells - 1;
        }
        if (waveGrid(waveGridx, searchIndex) < currDist && waveGrid(waveGridx, searchIndex) > 1) {
            waveGridy = searchIndex;
            path.waypoints.push_back(Eigen::Vector2d(x_min + (cellSize * waveGridx) + (cellSize / 2), 
                                                     y_min + (cellSize * waveGridy) + (cellSize / 2)));
            continue;
        }

        break;
    }

    path.waypoints.push_back(q_goal);

    return path;
}
