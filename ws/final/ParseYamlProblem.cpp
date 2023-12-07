#include "ParseYamlProblem.h"


amp::MultiAgentProblem2D buildEnvironment(const std::string& yamlFile) {
    amp::MultiAgentProblem2D problem;
    YAML::Node config = YAML::LoadFile(yamlFile);

    if (config["x_min"]) {
        problem.x_min = config["x_min"].as<double>();
    } else {
        LOG("YAML entry x_min missing from config file: Please include.");
        return problem;
    }

    if (config["x_max"]) {
        problem.x_max = config["x_max"].as<double>();
    } else {
        LOG("YAML entry x_max missing from config file: Please include.");
        return problem;
    }

    if (config["y_min"]) {
        problem.y_min = config["y_min"].as<double>();
    } else {
        LOG("YAML entry y_min missing from config file: Please include.");
        return problem;
    }

    if (config["y_max"]) {
        problem.y_max = config["y_max"].as<double>();
    } else {
        LOG("YAML entry y_max missing from config file: Please include.");
        return problem;
    }

    if (config["obstacles"]) {
        for (std::size_t i = 0; i < config["obstacles"].size(); i++) {
            std::vector<Eigen::Vector2d> vertices{};

            for (std::size_t j = 0; j < config["obstacles"][i]["vertices_ccw"].size(); j++) {
                double x = config["obstacles"][i]["vertices_ccw"][j]["x"].as<double>();
                double y = config["obstacles"][i]["vertices_ccw"][j]["y"].as<double>();
                vertices.push_back(Eigen::Vector2d(x, y));
            }

            problem.obstacles.push_back(amp::Obstacle2D(vertices));
        }
    }

    if (config["agents"]) {
        for (std::size_t i = 0; i < config["agents"].size(); i++) {
            amp::CircularAgentProperties properties{};

            properties.radius = config["agents"][i]["agent"]["radius"].as<double>();

            double q_init_x = config["agents"][i]["agent"]["q_init"]["x"].as<double>();
            double q_init_y = config["agents"][i]["agent"]["q_init"]["y"].as<double>();
            properties.q_init = Eigen::Vector2d(q_init_x, q_init_y);

            double q_goal_x = config["agents"][i]["agent"]["q_goal"]["x"].as<double>();
            double q_goal_y = config["agents"][i]["agent"]["q_goal"]["y"].as<double>();
            properties.q_goal = Eigen::Vector2d(q_goal_x, q_goal_y);

            problem.agent_properties.push_back(properties);
        }
    } else {
        LOG("YAML entry agents missing from config file: Please include.");
        return problem;
    }

    return problem;
}
