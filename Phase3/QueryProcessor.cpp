#include "QueryProcessor.hpp"

#include "QueryProcessor.hpp"
#include <iostream>
#include <vector>
#include <utility>
#include <unordered_map>

json delivery_route_optimization(const json &q, Graph &g) {
    
    int num_agents = 0;
    int depot_node = 0;
    unordered_map<int, std::pair<int, int>> orders_map;

    
    num_agents = q["fleet"]["num_delievery_guys"].get<int>();
    depot_node = q["fleet"]["depot_node"].get<int>();

    for (const auto& order_json : q["orders"]) {
        int order_id = order_json["order_id"].get<int>();
        int pickup_node = order_json["pickup"].get<int>();
        int dropoff_node = order_json["dropoff"].get<int>();
        
        orders_map[order_id] = {pickup_node, dropoff_node};
    }
    
    double total_delivery_time = 0.0;
    vector<pair<vector<int>, vector<int>>> assignments_result;
    
        
    assignments_result = g.delivery_route(
        num_agents, 
        depot_node, 
        orders_map, 
        total_delivery_time
    );
    
    json output;
    json assignments_array = json::array();

    for (size_t i = 0; i < assignments_result.size(); ++i) {
        const auto& assignment = assignments_result[i];
        
        json driver_assignment;
        driver_assignment["driver_id"] = i;
        
        driver_assignment["route"] = assignment.first; 
        
        driver_assignment["order_ids"] = assignment.second; 
        
        assignments_array.push_back(driver_assignment);
    }

    output["assignments"] = assignments_array;
    output["metrics"]["total_delivery_time_s"] = total_delivery_time;

    return output;
}