 #include "graph.hpp"
#include <unordered_set>
#include <unordered_map>
Graph::Graph():V(0){}

double Graph::euclideanDistance(int u, int v){
    double lat1 = nodes[u]->lat;
    double lon1 = nodes[u]->lon;
    double lat2 = nodes[v]->lat;
    double lon2 = nodes[v]->lon;
    return sqrt((lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2));
}
 
pair<vector<int>, double> Graph::shortestPath_minTime(int source, int destination,
                    vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool &possible){

    vector<double> dist(V, numeric_limits<double>::max());
    vector<int> prev(V, -1);
    vector<bool> visited(V, false);

    unordered_map<int, bool> forbidden_node_map;
    for(int fn: forbidden_nodes) forbidden_node_map[fn] = true;

    unordered_map<string, bool> forbidden_road_map;
    for(const string& frt: forbidden_road_types) forbidden_road_map[frt] = true;

    if(forbidden_node_map.count(source) || forbidden_node_map.count(destination)){
        possible = false;
        return {{}, -1};
    }

    dist[source] = 0.0;
    using PDI = pair<double,int>;
    priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
    pq.push({0.0, source});

    while(!pq.empty()){
        int u = pq.top().second; pq.pop();
        if(visited[u] || forbidden_node_map.count(u)) continue;
        visited[u] = true;

        if(u == destination) break;

        for(const auto& [v, e] : adj[u]){
            if(visited[v] || forbidden_node_map.count(v)) continue;
            if(e->blocked) continue;
            if(forbidden_road_map.count(e->road_type)) continue;

            double weight = e->avg_time;
            if(dist[u] + weight < dist[v]){
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    if(dist[destination] == numeric_limits<double>::max()){
        possible = false;
        return {{}, -1};
    }

    vector<int> path;
    for(int at = destination; at != -1; at = prev[at]) path.push_back(at);
    reverse(path.begin(), path.end());

    possible = true;
    return {path, dist[destination]};
}

 vector<vector<int>> Graph::nearestSeedClustering(int no_agents,unordered_map<int,pair<int,int>> orders){
        vector<int> order_ids;
        vector<pair<double,double>> centers;

    for (const auto& kv : orders) {
        int order_id = kv.first;
        int pickup_node = kv.second.first;
        int dropoff_node = kv.second.second;
        pair<double,double> p = {nodes[pickup_node]->lat,nodes[pickup_node]->lon};
        pair<double,double> d = {nodes[dropoff_node]->lat,nodes[dropoff_node]->lon};
        pair<double,double> center = {(p.first + d.first) / 2.0, (p.second + d.second) / 2.0};
        order_ids.push_back(order_id);
        centers.push_back(center);
    }
         std::vector<std::vector<int>> clusters(no_agents);
    int m = (int)centers.size();
    if (no_agents >= m) {
        for (int i = 0; i < m; i++) {
            clusters[i].push_back(order_ids[i]);
        }
        return clusters;
    }

    vector<int> sorted_indices(m);
    for (int i = 0; i < m; i++) sorted_indices[i] = i;
    sort(sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
        return centers[a].first < centers[b].first;
    });

    vector<pair<double,double>> seeds(no_agents);
    for (int k = 0; k < no_agents; k++) {
        seeds[k] = centers[sorted_indices[k * m / no_agents]];
    }

    for (int i = 0; i < m; i++) {
        double best_dist = std::numeric_limits<double>::max();
        int best_cluster = 0;
        for (int k = 0; k < no_agents; k++) {
             double lat1 = centers[i].first;
            double lon1 = centers[i].second;
            double lat2 = seeds[k].first;
            double lon2 = seeds[k].second;
            double dist = sqrt((lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2));
            if (dist < best_dist) {
                best_dist = dist;
                best_cluster = k;
            }
        }
        clusters[best_cluster].push_back(order_ids[i]);
    }

    return clusters;
 }


vector<int> Graph::buildGreedyRoute(int depot,unordered_map<int,pair<int,int>> orders,int no_agents,vector<int>cluster){
    unordered_set<int> available_pickups;
    unordered_set<int> available_deliveries;
   unordered_map<int, int> pickup_to_delivery;
    for (const auto& order_id : cluster) {
        available_pickups.insert(orders[order_id].first);
         pickup_to_delivery[orders[order_id].first] = orders[order_id].second;
    }

    std::vector<int> route;
    int curr = depot;
    route.push_back(depot);

      if (available_pickups.empty()) return route;

    while (!available_pickups.empty() || !available_deliveries.empty()) {
        double best_dist = std::numeric_limits<double>::max();
        int next_node = -1;
         bool take_pickup = false;

        for (int node : available_pickups) {
            double dist = euclideanDistance(curr, node);
            if (dist < best_dist) {
                best_dist = dist;
                next_node = node;
                take_pickup = true;
            }
        }

        for (int node : available_deliveries) {
            double dist = euclideanDistance(curr, node);
            if (dist < best_dist) {
                best_dist = dist;
                next_node = node;
                take_pickup = false;
            }
        }

          if (next_node == -1) break;
         route.push_back(next_node);
        curr = next_node;

        if ( take_pickup && available_pickups.count(next_node)) {
            available_pickups.erase(next_node);
             available_deliveries.insert(pickup_to_delivery[next_node]);
        } else if (available_deliveries.count(next_node)){
                    available_deliveries.erase(next_node);}
        
        }

    return route;
  }

  double Graph::getShortestPathTravelTime(int start, int end){

     bool possible;
   auto [path, dist] = shortestPath_minDistance(start, end, {}, {}, possible);


    if (!possible || path.size() < 2) {
        return (start == end) ? 0.0 : std::numeric_limits<double>::max();
    }

    double total_travel_time = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        int u = path[i-1];
        int v = path[i];

        bool edge_found = false;
        // Look for edge u->v in adjacency
        for (const auto& [adj_node, edge] : adj[u]) {
            if (adj_node == v) {
                total_travel_time += edge->avg_time;
                edge_found = true;
                break;
            }
        }

        if (!edge_found) {
            return std::numeric_limits<double>::max();
        }
    }

    return total_travel_time;
  }

pair<double,double> Graph::computetime(vector<int> route, unordered_map<int,pair<int,int>> orders){
         double current_time = 0.0;

    std::unordered_map<int, int> deliveryNodeToOrder;
    for (const auto& kv : orders) {
        deliveryNodeToOrder[kv.second.second] = kv.first;
    }

    std::unordered_map<int, double> completion_times;
    if (route.empty()) return {0.0, 0.0};
    int prev_node = route[0];

    for (size_t i = 1; i < route.size(); i++) {
        int curr_node = route[i];
        double dt = getShortestPathTravelTime(prev_node, curr_node);
         if (!isfinite(dt)) {
                return {numeric_limits<double>::infinity(), numeric_limits<double>::infinity()};}
            
        current_time += dt;

        if (deliveryNodeToOrder.count(curr_node)) {
            int order_id = deliveryNodeToOrder[curr_node];
            completion_times[order_id] = current_time;
        }

        prev_node = curr_node;
    }

    double sum_completion = 0.0, max_completion = 0.0;
    for (auto& kv : completion_times) {
        sum_completion += kv.second;
        if (kv.second > max_completion) max_completion = kv.second;
    }
    return {sum_completion, max_completion};

     }



  vector<pair<vector<int>,vector<int>>> Graph::delivery_route(int no_agents, int depot_node, unordered_map<int,pair<int,int>> orders, double& total_time){
        total_time = 0.0;

    vector<vector<int>> clusters = nearestSeedClustering(no_agents, orders);

    vector<pair<vector<int>, vector<int>>> result;
    unordered_map<int,int> pickupOfDelivery;

     for (auto& kv : orders) {
        int oid = kv.first;
        int p = kv.second.first;
        int d = kv.second.second;
        pickupOfDelivery[d] = p;
    }

    for (auto& cluster : clusters) {

        if (cluster.empty()) {
            result.push_back({{}, {}});
            continue;
        }

        vector<int> route = buildGreedyRoute(depot_node, orders, no_agents, cluster);
        auto [sum_c, max_c] = computetime(route, orders);

        total_time += sum_c;
        result.push_back({route, cluster});
    }

    return result;
  }