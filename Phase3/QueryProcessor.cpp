#include "QueryProcessor.hpp"

json schedule_deliveries(const json &q, Graph &g) {
    json output;
    json assignments = json::array();

    int numDrivers = q["fleet"]["num_delievery_guys"];
    int depot = q["fleet"]["depot_node"];

    const auto& orders = q["orders"];

    // For simplicity, assign all orders to driver 0
    json assignment;
    assignment["driver_id"] = 0;

    std::vector<int> route;
    route.push_back(depot);

    double total_time = 0.0;

    int current = depot;
    json order_ids = json::array();

    for (auto &order : orders) {
        int pickup = order["pickup"];
        int dropoff = order["dropoff"];

        // path to pickup
        auto [path1, time1] = g.shortestPathWithTime(current, pickup);
        route.insert(route.end(), path1.begin()+1, path1.end());
        total_time += time1;
        current = pickup;

        // path to dropoff
        auto [path2, time2] = g.shortestPathWithTime(current, dropoff);
        route.insert(route.end(), path2.begin()+1, path2.end());
        total_time += time2;
        current = dropoff;

        order_ids.push_back(order["order_id"]);
    }

    assignment["route"] = route;
    assignment["order_ids"] = order_ids;

    assignments.push_back(assignment);
    output["assignments"] = assignments;

    output["metrics"]["total_delivery_time_s"] = total_time;

    return output;
}
