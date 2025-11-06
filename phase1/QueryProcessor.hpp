#ifndef QUERYPROCESSOR_HPP
#define QUERYPROCESSOR_HPP

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "graph.hpp"

json shortest_path(const json &q, Graph &g);
json knn(const json &q, Graph &g);
json removeEdge(const json &q, Graph &g);
json modify_edge(const json &q, Graph &g);

#endif 