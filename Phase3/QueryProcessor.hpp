#pragma once

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "graph.hpp"

json schedule_deliveries(const json &q, Graph &g);
