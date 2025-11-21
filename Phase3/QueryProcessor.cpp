#include "QueryProcessor.hpp"

json schedule_deliveries(const json &q, Graph &g) {
    json output;
    json assignments = json::array();

    int numDrivers = q["fleet"]["num_delivery_guys"];
    int depot = q["fleet"]["depot_node"];
    const auto& orders = q["orders"];

    vector<vector<int>> driver_orders(numDrivers);

    // Round-robin assignment
    int i = 0;
    for (auto &order : orders) {
        driver_orders[i % numDrivers].push_back(order["order_id"]);
        i++;
    }

    double total_time_all = 0.0;

    for (int d = 0; d < numDrivers; ++d) {
        json assignment;
        assignment["driver_id"] = d;

        vector<int> route;
        route.push_back(depot);
        double total_time = 0.0;
        int current = depot;
        json order_ids = json::array();

        for (auto &order_id : driver_orders[d]) {
            const auto &order = orders[order_id.get<int>()]; // assuming order_id is index
            int pickup = order["pickup"];
            int dropoff = order["dropoff"];

            bool possible;
            auto [path1, time1] = g.shortestPath_minTime(current, pickup, {}, {}, possible);
            if (!possible) continue;

            route.insert(route.end(), path1.begin()+1, path1.end());
            total_time += time1;
            current = pickup;

            auto [path2, time2] = g.shortestPath_minTime(current, dropoff, {}, {}, possible);
            if (!possible) continue;

            route.insert(route.end(), path2.begin()+1, path2.end());
            total_time += time2;
            current = dropoff;

            order_ids.push_back(order["order_id"]);
        }

        assignment["route"] = route;
        assignment["order_ids"] = order_ids;
        assignments.push_back(assignment);

        total_time_all += total_time;
    }

    output["assignments"] = assignments;
    output["metrics"]["total_delivery_time_s"] = total_time_all;

    return output;
}
