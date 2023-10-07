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

    ManipulatorState jointAngles{};
    std::vector<Eigen::Vector2d> jointLocations{};
    bool isCollision = false;

    for (int i = 0; i < numx0Cells; i++) {
        jointAngles.push_back(i * CELL_SIZE);
        jointLocations.push_back(manipulator.getJointLocation(jointAngles, 0));
        jointAngles.push_back(0.0);
        jointLocations.push_back(manipulator.getJointLocation(jointAngles, 1));
        jointAngles.pop_back();

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
                jointAngles.push_back(j * CELL_SIZE);
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

                jointAngles.pop_back();
                jointLocations.pop_back();
            }
        }

        jointAngles.clear();
        jointLocations.clear();
    }

    return cSpace;
}

bool MyGridCSpace2D::inCollision(double x0, double x1) const {
    int x0Index = static_cast<int>(std::floor(x0 / CELL_SIZE)) % this->size().first;
    int x1Index = static_cast<int>(std::floor(x1 / CELL_SIZE)) % this->size().second;
    bool collisionValue = (*this)(x0Index, x1Index);
    return collisionValue;
}
