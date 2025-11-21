#pragma once

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "graph.hpp"

json delivery_route_optimization(const json &q, Graph &g);