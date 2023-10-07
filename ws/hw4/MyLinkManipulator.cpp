# include "MyLinkManipulator.h"

using namespace amp;

MyLinkManipulator::MyLinkManipulator(const std::vector<double>& linkLengths, 
                                     const ManipulatorState& config, 
                                     const Eigen::Vector2d& baseLocation)
                                     : LinkManipulator2D(baseLocation, linkLengths) {
    for (int i = 0; i < nLinks() + 1; i++) {
        jointLocations.push_back(getJointLocation(config, i));
    }
    std::cout << "End Effector Location: (" << jointLocations.back()[0] <<
                  ", " << jointLocations.back()[1] << ")" << std::endl;
    Visualizer::makeFigure(*this, config);
    Visualizer::showFigures();
}

MyLinkManipulator::MyLinkManipulator(const std::vector<double>& linkLengths, 
                                     const Eigen::Vector2d& endEffectPos, 
                                     const Eigen::Vector2d& baseLocation)
                                     : LinkManipulator2D(baseLocation, linkLengths) {
    ManipulatorState jointConfigurations = getConfigurationFromIK(endEffectPos);
    std::cout << "Inverse Kinematics Configuration 1" << std::endl;
    for (int i = 0; i < jointConfigurations.size(); i++) {
        std::cout << "Theta " << i << ": " << jointConfigurations[i] << " radians" << std::endl;
    }
    Visualizer::makeFigure(*this, jointConfigurations);

    ManipulatorState invJointConfigurations{};
    std::cout << "Inverse Kinematics Configuration 2" << std::endl;
    for (int i = 0; i < jointConfigurations.size(); i++) {
        invJointConfigurations.push_back((jointConfigurations[i] * -1.0) + (2 * M_PI));
        std::cout << "Theta " << i << ": " << invJointConfigurations[i] << " radians" << std::endl;
    }
    Visualizer::makeFigure(*this, invJointConfigurations);

    Visualizer::showFigures();
}

MyLinkManipulator::~MyLinkManipulator() {
    jointLocations.clear();
}

Eigen::Vector2d MyLinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const {
    Eigen::Matrix<double, 3, 3> transformationMatrix{{1.0, 0.0, 0.0}, 
                                                     {0.0, 1.0, 0.0}, 
                                                     {0.0, 0.0, 1.0}};
    Eigen::Vector3d localOrigin{0.0, 0.0, 1.0};
    Eigen::Vector3d baseTranslation{getBaseLocation()[0], getBaseLocation()[1], 0.0};
    double rotation = 0.0;
    double translation = 0.0;

    for (int i = 0; i < joint_index + 1; i++) {
        if (i == 0) {
            rotation = state[i];
            translation = 0.0;
        } else if (i == nLinks()) {
            rotation = 0.0;
            translation = getLinkLengths()[i - 1];
        } else {
            rotation = state[i];
            translation = getLinkLengths()[i - 1];
        }

        transformationMatrix *= getTransformationMatrix(rotation, translation);
    }

    Eigen::Vector3d holder = (transformationMatrix * localOrigin) + baseTranslation;

    return Eigen::Vector2d(holder[0], holder[1]);
}

ManipulatorState MyLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    ManipulatorState jointAngles{};
    ManipulatorState tmpJointAngles{};
    Eigen::Vector2d baseLocation = getBaseLocation();
    Eigen::Vector2d currentJointLocation{};
    const std::vector<double> linkLengths = getLinkLengths();
    double endPosDistance = 0.0;
    double endPosAngle = 0.0;
    double cumulativeAngle = 0.0;
    double armReach = reach();
    int bendIndex = 1;

    endPosDistance = getEndEffectorDistance(baseLocation, end_effector_location);

    if (endPosDistance == armReach) {
        bendIndex = nLinks() - 1;
    } else {
        for (int i = nLinks() - 1; i > 0; i--) {
            armReach -= linkLengths[i];
            if (armReach <= endPosDistance) {
                bendIndex = i;
                break;
            }
        }
    }

    for (int i = 0; i < nLinks(); i++) {
        tmpJointAngles = jointAngles;
        tmpJointAngles.push_back(0.0);

        currentJointLocation = getJointLocation(tmpJointAngles, i);
        endPosAngle = getEndEffectorAngle(currentJointLocation, end_effector_location);
        endPosDistance = getEndEffectorDistance(currentJointLocation, end_effector_location);

        double links1 = 0.0;
        double links2 = 0.0;
        if (i < bendIndex) {
            links1 = std::accumulate(linkLengths.begin() + i, linkLengths.begin() + bendIndex, 0.0);
            links2 = std::accumulate(linkLengths.begin() + bendIndex, linkLengths.end(), 0.0);
        } else {
            links1 = std::accumulate(linkLengths.begin() + i, linkLengths.end(), 0.0);
        }

        double angle = acos((pow(links1, 2) + pow(endPosDistance, 2) - 
                             pow(links2, 2)) / (2 * links1 * endPosDistance));
        if (angle < 0.0) {
            angle += 2 * M_PI;
        }

        double jointAngle = angle + endPosAngle - cumulativeAngle;
        if (jointAngle < 0.0) {
            jointAngle += 2 * M_PI;
        }
        jointAngles.push_back(jointAngle);
        cumulativeAngle += jointAngles.back();

        tmpJointAngles.clear();
    }

    return jointAngles;
}

Eigen::Matrix<double, 3, 3> MyLinkManipulator::getTransformationMatrix(const double& rotation, 
                                                                       const double& translation) const {
    Eigen::Matrix<double, 3, 3> T{{cos(rotation), -sin(rotation), translation}, 
                                  {sin(rotation), cos(rotation), 0}, 
                                  {0, 0, 1}};
    return T;
}

double MyLinkManipulator::getEndEffectorDistance(const Eigen::Vector2d& jointLocation, 
                                                 const Eigen::Vector2d& endEffectorLocation) const {
    return sqrt(pow(jointLocation[0] - endEffectorLocation[0], 2) + 
                pow(jointLocation[1] - endEffectorLocation[1], 2));
}

double MyLinkManipulator::getEndEffectorAngle(const Eigen::Vector2d& jointLocation, 
                                              const Eigen::Vector2d& endEffectorLocation) const {
    return atan2(endEffectorLocation[1] - jointLocation[1], 
                 endEffectorLocation[0] - jointLocation[0]);
}
