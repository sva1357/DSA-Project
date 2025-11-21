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
    double time_remaining_ms = time_budget_ms-0.5;
    int done_queries = 0;
    for(const auto& query: q["queries"]){

        int source = query["source"];
        int destination = query["target"];
        double time_for_query_ms = time_remaining_ms / (q["queries"].size()- done_queries);
        double start_time = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
        double approx_distance = g.approxShortestPath(source, destination, time_for_query_ms, acceptable_error_pct);
        double end_time = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
        time_remaining_ms -= (end_time - start_time);
        json query_result;
        query_result["source"] = source;
        query_result["target"] = destination;
        query_result["approx_shortest_distance"] = approx_distance;
        output["distances"].push_back(query_result);
        done_queries++;
    }
    output["id"] = id;
    return output;
}
