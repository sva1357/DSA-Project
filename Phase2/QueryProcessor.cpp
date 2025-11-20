#include "QueryProcessor.hpp"

json k_shortest_paths(const json &q, Graph &g) {
    int id = q["id"];
    int source = q["source"];
    int target = q["target"];
    int K = q["k"];
    json output;
    vector<pair<vector<int>, double>> result = g.kShortestPaths_exact(source, target, K);
    output["id"] = id;
    output["paths"] = json::array();
    for (const auto& [path, cost] : result) {
        json path_info;
        path_info["path"] = path;
        path_info["length"] = cost;
        output["paths"].push_back(path_info);
    }
    return output; 
}

json k_shortest_paths_heuristic(const json &q, Graph &g){
    int id = q["id"];
    int source = q["source"];
    int target = q["target"];
    int K = q["k"];
    int threshold = q["overlap_threshold"];
    json output;
    vector<pair<vector<int>, double>> result = g.kShortestPaths_Heuristic(source, target, K, threshold);
    output["id"] = id;
    output["paths"] = json::array();
    for (const auto& [path, cost] : result) {
        json path_info;
        path_info["path"] = path;
        path_info["length"] = cost;
        output["paths"].push_back(path_info);
    }
    return output;
}
json approx_shortest_path(const json &q, Graph &g){
    int id = q["id"];
    json output;
    double time_budget_ms = q["time_budget_ms"];
    double acceptable_error_pct = q["acceptable_error_pct"];
    for(const auto& query: q["queries"]){
        int source = query["source"];
        int destination = query["target"];
        double approx_distance = g.approxShortestDistance(source, destination, time_budget_ms, acceptable_error_pct);
        json query_result;
        query_result["source"] = source;
        query_result["target"] = destination;
        query_result["approx_shortest_distance"] = approx_distance;
        output["distances"].push_back(query_result);
    }
    output["id"] = id;
    return output;
}
