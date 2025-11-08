#pragma once

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "graph.hpp"

json k_shortest_paths(const json &q, Graph &g);
json k_shortest_paths_heuristic(const json &q, Graph &g);
json approx_shortest_path(const json &q, Graph &g);
