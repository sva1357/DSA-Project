#include "QueryProcessor.hpp"

json shortest_path(const json &q, Graph &g){
    int id = q["id"];
    int src_id = q["source"];
    int target_id = q["target"];
    string mode = q["mode"];
    vector<int> forbidden_nodes;
    vector<string> forbidden_road_types;

    if (q.contains("constraints")) {
        auto constraints = q["constraints"];

        forbidden_nodes = constraints.value("forbidden_nodes", vector<int>{});
        forbidden_road_types = constraints.value("forbidden_road_types", vector<string>{});
    } else {
        forbidden_nodes = {};
        forbidden_road_types = {};
    }

    bool possible;
    json output;

    if(mode == "distance"){
        pair<vector<int>,double> result = g.shortestPath_minDistance(src_id,target_id,forbidden_nodes,forbidden_road_types,possible);
        
        output["id"] = id;
        output["possible"] = possible;
        if(possible == false){
            return output;
        }
        output["minimum_time/minimum_distance"] = result.second;
        output["path"] = result.first;
        return output;
    }

    else{
        pair<vector<int>,double> result = g.shortestPath_minTime(src_id,target_id,forbidden_nodes,forbidden_road_types,possible);
        output["id"] = id;
        output["possible"] = possible;
        if(possible == false){
            return output;
        }
        output["minimum_time/minimum_distance"] = result.second;
        output["path"] = result.first;
        return output;
    }

}

json knn(const json &q, Graph &g){
    int id = q["id"];
    string poi = q["poi"];
    double query_lat = q["query_point"]["lat"];
    double query_lon = q["query_point"]["lon"];
    int K = q["k"];
    string metric = q["metric"];

    vector<int> result = g.knn(poi, query_lat, query_lon, K, metric);

    json output;
    output["id"] = id;
    output["nodes"] = result;
    return output;
}

json removeEdge(const json &q, Graph &g){
    int id = q["id"];
    int edge_id = q["edge_id"];

    bool success = g.removeEdge(edge_id);

    json output;
    output["id"] = id;
    output["done"] = success;
    return output;
}

json modify_edge(const json &q, Graph &g){
    int id = q["id"];
    int edge_id = q["edge_id"];

    bool change_time = q["patch"].contains("average_time");
    double new_avg_time;
    if(change_time) {
        new_avg_time = q["patch"]["average_time"];
    }
    bool change_speed_profile = q["patch"].contains("speed_profile");
    vector<double> new_speed_profile;
    if(change_speed_profile) {
        new_speed_profile = q["patch"]["speed_profile"].get<vector<double>>();
    }
    bool change_length = q["patch"].contains("length");
    double new_length;
    if(change_length) {
        new_length = q["patch"]["length"];
    }
    bool change_road_type = q["patch"].contains("road_type");
    string new_road_type;
    if(change_road_type) {
        new_road_type = q["patch"]["road_type"];
    }
    g.modifyEdge(edge_id, change_time, new_avg_time, change_speed_profile, new_speed_profile, change_length, new_length, change_road_type, new_road_type);

    json output;
    output["id"] = id;
    output["done"] = true;
    return output;
}