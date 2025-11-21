#include "QueryProcessor.hpp"
#include "graph.hpp" // make sure clusterOrdersIterative and buildRoute are declared

json schedule_deliveries(const json &q, Graph &g) {
    json output;
    json assignments = json::array();

    int numDrivers = q["fleet"]["num_delivery_guys"];
    int depot = q["fleet"]["depot_node"];
    const auto& orders_json = q["orders"];

    // Step 1: Convert JSON orders to Graph::Order
    vector<Order> orders;
    for (auto &o : orders_json) {
        Order order;
        order.pickup = o["pickup"];
        order.delivery = o["dropoff"];
        // assume Graph nodes store lat/lon
        Node* pickupNode = g.getNode(order.pickup);  
        Node* deliveryNode = g.getNode(order.delivery);
        order.pickupNode = {pickupNode->lat, pickupNode->lon, order.pickup};
        order.deliveryNode = {deliveryNode->lat, deliveryNode->lon, order.delivery};
        order.center = {(order.pickupNode.x + order.deliveryNode.x)/2.0,
                        (order.pickupNode.y + order.deliveryNode.y)/2.0,
                        -1};
        orders.push_back(order);
    }

    // Step 2: Cluster orders among drivers
    vector<vector<Order>> clusters = g.clusterOrdersIterative(orders, numDrivers);

    // Step 3: Build route for each driver
    double total_time_all = 0.0;

    for (int d = 0; d < numDrivers; ++d) {
        json assignment;
        assignment["driver_id"] = d;

        // Build distance matrix for this cluster (depot + pickups + deliveries)
        vector<DeliveryNode> nodes;
        nodes.push_back({g.getNode(depot)->lat, g.getNode(depot)->lon, depot});
        for (auto &o : clusters[d]) {
            nodes.push_back(o.pickupNode);
            nodes.push_back(o.deliveryNode);
        }

        int N = nodes.size();
        vector<vector<double>> dist(N, vector<double>(N, 0.0));
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                dist[i][j] = Graph::euclidean(nodes[i], nodes[j]);

        // Build greedy route
        vector<int> route = g.buildRoute(0, clusters[d], dist);

        assignment["route"] = route;

        // Assign order IDs
        json order_ids = json::array();
        for (auto &o : clusters[d]) order_ids.push_back(o.pickup); // or original order id
        assignment["order_ids"] = order_ids;

        // Compute approximate total time
        double driver_time = 0.0;
        int current = depot;
        for (int node_id : route) {
            if (node_id == current) continue;
            driver_time += g.euclideanDistance(current, node_id); // placeholder for actual travel time
            current = node_id;
        }

        total_time_all += driver_time;
        assignments.push_back(assignment);
    }

    output["assignments"] = assignments;
    output["metrics"]["total_delivery_time_s"] = total_time_all;

    return output;
}
