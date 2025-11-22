#include "graph.hpp"
#include <unordered_set>
#include <unordered_map>
Graph::Graph():V(0){}

void Graph::addNode(int id, double lat, double lon, vector<string> pois){
    Node* n=new Node(id,lat,lon,pois);
    nodes.push_back(n);
    adj.push_back({});
    V++;
}

void Graph::addEdge(int id, int u, int v, double length, double avg_time, vector<double> speed_profile,
                bool oneway, string road_type){
    Edge* e=new Edge(id,u,v,length,avg_time,speed_profile,oneway,road_type);
    edges[id]=e;
    adj[u].push_back({v,e});
    if(!oneway){
        adj[v].push_back({u,e});
    }
}

bool Graph::removeEdge(int id){
    if(edges.find(id)==edges.end()) return false;
    Edge* e=edges[id];
    if(e->blocked) return false;
    e->blocked=true;
    return true;
}

void Graph::modifyEdge(int id, bool change_time, double new_avg_time, bool change_speed_profile, 
                    vector<double> new_speed_profile, bool change_length, double new_length, bool change_road_type, string new_road_type){
    if(edges.find(id)==edges.end()) return;
    Edge* e=edges[id];
    if(change_time) e->avg_time=new_avg_time;
    if(change_speed_profile) e->speed_profile=new_speed_profile;
    if(change_length) e->len=new_length;
    if(change_road_type) e->road_type=new_road_type;
    e->blocked=false;
}

double Graph::euclideanDistance(int u, int v){
    double lat1 = nodes[u]->lat;
    double lon1 = nodes[u]->lon;
    double lat2 = nodes[v]->lat;
    double lon2 = nodes[v]->lon;
    return sqrt((lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2));
}

pair<vector<int>,double> Graph::shortestPath_minDistance(int source, int destination, vector<int> forbidden_nodes,
                                                                vector<string> forbidden_road_types, bool& possible){
    
    vector<double> dist(V, numeric_limits<double>::max());
    vector<int> prev(V, -1);
    vector<bool> visited(V, false);

    unordered_map<int, bool> forbidden_node_map;
    for (int fn : forbidden_nodes) forbidden_node_map[fn] = true;

    unordered_map<string, bool> forbidden_road_type_map;
    for (const string& frt : forbidden_road_types) forbidden_road_type_map[frt] = true;

    dist[source] = 0.0;
    using PDI = pair<double, int>;
    priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
    pq.push({0.0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (visited[u] || forbidden_node_map.count(u)) continue;
        visited[u] = true;
        if (u == destination) break;

        for (const auto& [v, e] : adj[u]) {
            if (visited[v] || forbidden_node_map.count(v)) continue;
            if (e->blocked) continue;
            if (forbidden_road_type_map.count(e->road_type)) continue;
            double weight = e->len;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    vector<int> path;
    if (dist[destination] == numeric_limits<double>::max()) {
        possible = false;
        return {{}, -1};
    }
    for (int at = destination; at != -1; at = prev[at]) path.push_back(at);

    reverse(path.begin(), path.end());
    possible = true;
    return {path,dist[destination]};
    
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

vector<pair<vector<int>, double>> Graph::shortestPath_allTimes(int source){
    vector<pair<vector<int>,double>> result;
    result.resize(V);
    vector<double> dist(V, numeric_limits<double>::max());
    vector<int> prev(V, -1);
    vector<bool> visited(V, false);
    dist[source] = 0.0;
    using PDI = pair<double, int>;
    priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
    pq.push({0.0, source});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (visited[u]) continue;
        visited[u] = true;

        for (const auto& [v, e] : adj[u]) {
            if (visited[v]) continue;
            if (e->blocked) continue;
            double weight = e->avg_time;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    for(int destination=0; destination<V; destination++){
        vector<int> path;
        if(dist[destination]==numeric_limits<double>::max()){
            result[destination]={path,-1};
            continue;
        }
        for(int at=destination; at!=-1; at=prev[at]) path.push_back(at);
        reverse(path.begin(), path.end());
        result[destination]={path, dist[destination]};
    }
    return result;
}

vector<vector<int>> Graph::nearestSeedClustering(int no_agents, unordered_map<int,pair<int,int>>& orders){
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


vector<int> Graph::buildGreedyRoute_1(int depot, unordered_map<int, pair<int,int>>& orders, vector<int>& cluster, double& sum_c, double& max_c) {
    unordered_set<int> available_pickups;
    unordered_map<int,int> available_deliveries;
    unordered_map<int,int> pickup_to_delivery;

  
    for (int order_id : cluster) {
        int p = orders[order_id].first;
        int d = orders[order_id].second;
        available_pickups.insert(p);
        pickup_to_delivery[p] = d;
    }

    vector<int> full_route;
    int curr_node = depot;
    full_route.push_back(depot);
    double current_time = 0.0;
    if (available_pickups.empty()) return full_route;

    while (!available_pickups.empty() || !available_deliveries.empty()) {
        double best_time = numeric_limits<double>::max();
        int next_node = -1;
        bool take_pickup = false;
        vector<int> best_path;

        auto allTimes = shortestPath_allTimes(curr_node);
        
        for (int p : available_pickups) {
            bool possible = false;
            auto [path, time] = allTimes[p];
            if(time!=-1) possible=true;
            if (possible && time < best_time) {
                best_time = time;
                next_node = p;
                take_pickup = true;
                best_path = path;
            }
        }

        for (auto [d,x] : available_deliveries) {
            bool possible = false;
            auto [path, time] = allTimes[d];
            if(time!=-1) possible=true;
            if (possible && time < best_time) {
                best_time = time;
                next_node = d;
                take_pickup = false;
                best_path = path;
            }
        }

        if (next_node == -1 || best_path.size() < 2) break;

      
        full_route.insert(full_route.end(), best_path.begin() + 1, best_path.end());
        curr_node = next_node;
        current_time += best_time;

        if (take_pickup) {
            available_pickups.erase(next_node);
            int delivery_node = pickup_to_delivery[next_node];
            if(available_deliveries.find(delivery_node) == available_deliveries.end()) {
                available_deliveries[delivery_node] = 1;
            } else {
                available_deliveries[delivery_node]++;
            }
        } else {
            sum_c += current_time* available_deliveries[next_node];
            if (current_time > max_c) max_c = current_time;
            available_deliveries.erase(next_node);
        }
    }

    return full_route;
}

vector<pair<vector<int>,vector<int>>> Graph::delivery_route_near(int no_agents, int depot_node, unordered_map<int,pair<int,int>>& orders, double& total_time){
    total_time = 0.0;

    vector<vector<int>> clusters = nearestSeedClustering(no_agents, orders);

    vector<pair<vector<int>, vector<int>>> result;
    unordered_map<int,int> pickupOfDelivery;

    for (auto& kv : orders) {
        int p = kv.second.first;
        int d = kv.second.second;
        pickupOfDelivery[d] = p;
    }

    for (auto& cluster : clusters) {

        if (cluster.empty()) {
            result.push_back({{}, {}});
            continue;
        }
        double sum_c = 0.0, max_c = 0.0;

        vector<int> route = buildGreedyRoute_1(depot_node, orders, cluster, sum_c, max_c);

        total_time += sum_c;
        result.push_back({route, cluster});
    }

    return result;
}
    
pair<double,double> Graph::computetime(vector<int> route, unordered_map<int,pair<int,int>> orders){
         double current_time = 0.0;

    std::unordered_map<int, int> deliveryNodeToOrder;
    for (const auto& kv : orders) {
        deliveryNodeToOrder[kv.second.second] = kv.first;
    }

    std::unordered_map<int, double> completion_times;
    if (route.size() <= 1) return {0.0, 0.0};
    int u = route[0];

    for (size_t i = 1; i < route.size(); i++) {
          int v = route[i];

        bool edge_found = false;
        for (auto& [adj_node, edge] : adj[u]) {
            if (adj_node == v) {
                current_time += edge->avg_time;
                edge_found = true;
                break;
            }
        }
        if (!edge_found) return {numeric_limits<double>::infinity(), numeric_limits<double>::infinity()};

        if (deliveryNodeToOrder.count(v)) {
            int order_id = deliveryNodeToOrder[v];
            completion_times[order_id] = current_time;
        }

        u = v;
    }

    double sum_completion = 0.0, max_completion = 0.0;
    for (auto& kv : completion_times) {
        sum_completion += kv.second;
        if (kv.second > max_completion) max_completion = kv.second;
    }
    return {sum_completion, max_completion};

     }


vector<int> Graph::buildGreedyRoute(int depot, unordered_map<int, pair<int,int>> orders, vector<int> cluster) {
    unordered_set<int> available_pickups;
    unordered_set<int> available_deliveries;
    unordered_map<int,int> pickup_to_delivery;

  
    for (int order_id : cluster) {
        int p = orders[order_id].first;
        int d = orders[order_id].second;
        available_pickups.insert(p);
        pickup_to_delivery[p] = d;
    }

    vector<int> full_route;
    int curr_node = depot;
    full_route.push_back(depot);

    if (available_pickups.empty()) return full_route;

    while (!available_pickups.empty() || !available_deliveries.empty()) {
        double best_time = numeric_limits<double>::max();
        int next_node = -1;
        bool take_pickup = false;
        vector<int> best_path;

        for (int p : available_pickups) {
            bool possible = false;
            auto [path, time] = shortestPath_minTime(curr_node, p, {}, {}, possible);
            if (possible && time < best_time) {
                best_time = time;
                next_node = p;
                take_pickup = true;
                best_path = path;
            }
        }

        for (int d : available_deliveries) {
            bool possible = false;
            auto [path, time] = shortestPath_minTime(curr_node, d, {}, {}, possible);
            if (possible && time < best_time) {
                best_time = time;
                next_node = d;
                take_pickup = false;
                best_path = path;
            }
        }

        if (next_node == -1 || best_path.size() < 2) break;

      
        full_route.insert(full_route.end(), best_path.begin() + 1, best_path.end());
        curr_node = next_node;

        if (take_pickup) {
            available_pickups.erase(next_node);
            available_deliveries.insert(pickup_to_delivery[next_node]);
        } else {
            available_deliveries.erase(next_node);
        }
    }

    return full_route;
}

vector<vector<int>> Graph::kmeansPlusPlusClusters(
        int no_agents,
        unordered_map<int, pair<int,int>> orders)
{
    vector<int> order_ids;
    vector<pair<double,double>> centers;

    for (auto& kv : orders) {
        int oid = kv.first;
        int p = kv.second.first;
        int d = kv.second.second;

        double cx = (nodes[p]->lat + nodes[d]->lat) / 2.0;
        double cy = (nodes[p]->lon + nodes[d]->lon) / 2.0;

        order_ids.push_back(oid);
        centers.push_back({cx, cy});
    }

    int m = centers.size();
    vector<vector<int>> clusters(no_agents);

    if (no_agents >= m) {
        for (int i = 0; i < m; i++)
            clusters[i % no_agents].push_back(order_ids[i]);
        return clusters;
    }

    
    vector<bool> used(m, false);
    vector<pair<double,double>> seeds;

    double meanx = 0, meany = 0;
    for (auto& c : centers) {
        meanx += c.first;
        meany += c.second;
    }
    meanx /= m;
    meany /= m;

    int first = -1;
    double best = 1e18;

    for (int i = 0; i < m; i++) {
        double dx = centers[i].first - meanx;
        double dy = centers[i].second - meany;
        double d2 = dx*dx + dy*dy;
        if (d2 < best) {
            best = d2;
            first = i;
        }
    }

    used[first] = true;
    seeds.push_back(centers[first]);

    vector<double> minDistSq(m, 1e18);

    for (int k = 1; k < no_agents; k++) {

        auto& last = seeds.back();

        for (int i = 0; i < m; i++) {
            if (!used[i]) {
                double dx = centers[i].first - last.first;
                double dy = centers[i].second - last.second;
                double d2 = dx*dx + dy*dy;
                if (d2 < minDistSq[i]) minDistSq[i] = d2;
            }
        }

        int next = -1;
        double bestD = -1;

        for (int i = 0; i < m; i++) {
            if (!used[i] && minDistSq[i] > bestD) {
                bestD = minDistSq[i];
                next = i;
            }
        }

        used[next] = true;
        seeds.push_back(centers[next]);
    }

    for (int i = 0; i < m; i++) {
        double bestDist = 1e18;
        int bestCluster = 0;

        for (int k = 0; k < no_agents; k++) {
            double dx = centers[i].first - seeds[k].first;
            double dy = centers[i].second - seeds[k].second;
            double d2 = dx*dx + dy*dy;

            if (d2 < bestDist) {
                bestDist = d2;
                bestCluster = k;
            }
        }

        clusters[bestCluster].push_back(order_ids[i]);
    }

    return clusters;
}


vector<int> Graph::twoPointSwap(const vector<int>& route) {
    vector<int> r = route;
    int n = r.size();
    if (n < 5) return r;

    int i = rand() % (n-3) + 1;
    int j = rand() % (n-i-2) + (i+2);

    reverse(r.begin()+i, r.begin()+j);
    return r;
}

bool Graph::validPickupDelivery(const vector<int>& route,
     unordered_map<int,pair<int,int>>& orders)
{
    unordered_map<int,int> pos;
    for (int i = 0; i < route.size(); i++)
        pos[route[i]] = i;

    for (auto& kv : orders) {
        int p = kv.second.first;
        int d = kv.second.second;
        if (pos[p] > pos[d]) return false;
    }

    return true;
}

vector<int> Graph::shiftNode(const vector<int>& route) {
    vector<int> r = route;
    int n = r.size();
    int i = rand() % (n-1) + 1;

    int node = r[i];
    r.erase(r.begin()+i);

    int j = rand() % (n-1) + 1;
    r.insert(r.begin()+j, node);

    return r;
}


vector<pair<vector<int>,vector<int>>> 
Graph::delivery_route(int no_agents, 
                      int depot_node, 
                      unordered_map<int,pair<int,int>> orders, 
                      double& total_time)
{
    total_time = 0.0;
    auto clusters = kmeansPlusPlusClusters(no_agents, orders);
    vector<pair<vector<int>,vector<int>>> output;
    output.resize(no_agents);
    for (int k = 0; k < no_agents; k++) {

        vector<int> order_ids = clusters[k];

        vector<int> route = buildGreedyRoute(depot_node, orders, order_ids);

        auto [bestSum, bestMax] = computetime(route, orders);
        double bestCost = bestSum;              // MIN_SUM objective
        vector<int> bestRoute = route;

        auto start = chrono::high_resolution_clock::now();

        while (true) {
            auto now = chrono::high_resolution_clock::now();
            double elapsed = chrono::duration<double>(now - start).count();
            if (elapsed > 0.3) break;  

            vector<int> cand = 
                (rand() % 2 ? twoPointSwap(bestRoute)
                            : shiftNode(bestRoute));

            if (!validPickupDelivery(cand, orders))
                continue;

            auto [sumC, maxC] = computetime(cand, orders);
            double costC = sumC;  // MIN_SUM

            if (costC < bestCost) {
                bestCost = costC;
                bestRoute = cand;
            }
        }

        output[k] = { bestRoute, order_ids };

        total_time += bestCost;
    }

    return output;
}