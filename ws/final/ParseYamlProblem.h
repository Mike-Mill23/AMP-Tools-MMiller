#pragma once

#include "AMPCore.h"
#include "yaml-cpp/yaml.h"
#include <string.h>

amp::MultiAgentProblem2D buildEnvironment(const std::string& yamlFile);
