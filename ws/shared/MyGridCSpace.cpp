# include "MyGridCSpace.h"

using namespace amp;

std::unique_ptr<amp::GridCSpace2D> MyGridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, 
                                                                        const amp::Environment2D& env) {
    std::size_t numx0Cells = 2 * M_PI / CELL_SIZE;
    std::size_t numx1Cells = 2 * M_PI / CELL_SIZE;
    double x0min = 0.0;
    double x0max = 2 * M_PI;
    double x1min = 0.0;
    double x1max = 2 * M_PI;
    std::unique_ptr<amp::MyGridCSpace2D> cSpace = std::make_unique<amp::MyGridCSpace2D>(numx0Cells, numx1Cells, x0min, x0max, x1min, x1max);

    ManipulatorState jointAngles(manipulator.nLinks());
    std::vector<Eigen::Vector2d> jointLocations{};
    bool isCollision = false;

    for (int i = 0; i < numx0Cells; i++) {
        jointAngles[0] = (i * CELL_SIZE) + (CELL_SIZE / 2);
        jointLocations.push_back(manipulator.getJointLocation(jointAngles, 0));
        jointAngles[1] = 0.0;
        jointLocations.push_back(manipulator.getJointLocation(jointAngles, 1));

        for (auto& poly : env.obstacles) {
            isCollision = collisionLinePolygon(jointLocations, poly);
            if (isCollision) {
                break;
            }
        }

        if (isCollision) {
            for (int j = 0; j < numx1Cells; j++) {
                (*cSpace)(i, j) = true;
            }
        } else {
            for (int j = 0; j < numx1Cells; j++) {
                jointAngles[1] = (j * CELL_SIZE) + (CELL_SIZE / 2);
                jointLocations.push_back(manipulator.getJointLocation(jointAngles, 2));

                std::vector<Eigen::Vector2d> link2{jointLocations.begin() + 1, jointLocations.end()};

                for (auto& poly : env.obstacles) {
                    isCollision = collisionLinePolygon(link2, poly);
                    if (isCollision) {
                        break;
                    }
                }

                if (isCollision) {
                    (*cSpace)(i, j) = true;
                } else {
                    (*cSpace)(i, j) = false;
                }

                jointLocations.pop_back();
            }
        }

        jointLocations.clear();
    }

    return cSpace;
}

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    auto[numx0Cells, numx1Cells] = this->size();
    auto[x0min, x0max] = this->x0Bounds();
    auto[x1min, x1max] = this->x1Bounds();
    double x0cellSize = (x0max - x0min) / numx0Cells;
    double x1cellSize = (x1max - x1min) / numx1Cells;
    std::size_t x0Index = static_cast<std::size_t>(std::floor((x0 - x0min) / x0cellSize)) % numx0Cells;
    std::size_t x1Index = static_cast<std::size_t>(std::floor((x1 - x0min) / x1cellSize)) % numx1Cells;
    return std::make_pair(x0Index, x1Index);
}
