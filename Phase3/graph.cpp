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
    if(adj_matrix.size()<(size_t)V){
        adj_matrix.resize(V);
        for(int i=0;i<V;i++) adj_matrix[i].resize(V);
    }
    edges[id]=e;
    adj[u].push_back({v,e});
    adj_matrix[u][v]={v,e};
    if(!oneway){
        adj[v].push_back({u,e});
        adj_matrix[v][u]={u,e};
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


vector<int> Graph::buildGreedyRoute(int depot, unordered_map<int, pair<int,int>>& orders, vector<int>& cluster, double& sum_c, double& max_c) {
    unordered_map<int,vector<int>> available_pickups;
    unordered_map<int,int> available_deliveries;

  
    for (int order_id : cluster) {
        int p = orders[order_id].first;
        int d = orders[order_id].second;
        available_pickups[p].push_back(d);
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
        
        for (auto [p,x] : available_pickups) {
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
            for(auto d : available_pickups[next_node]){
                available_deliveries[d]++;
            }
            available_pickups.erase(next_node);
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

        vector<int> route = buildGreedyRoute(depot_node, orders, cluster, sum_c, max_c);
        auto res = computetime(route, orders, cluster);
        sum_c = res.first;
        max_c = res.second;

        total_time += sum_c;
        result.push_back({route, cluster});
    }

    return result;
}
    
pair<double,double> Graph::computetime(vector<int> route, unordered_map<int,pair<int,int>> orders,vector<int> order_ids){
     double current_time = 0.0;

    unordered_map<int,vector<int>> pickup_nodes;
    for(int oid : order_ids){
        int p = orders[oid].first;
        pickup_nodes[p].push_back(orders[oid].second);
    }
    unordered_map<int,int> delivery_nodes;

    double sum_completion = 0.0, max_completion = 0.0;

    if (route.size() <= 1) return {0.0, 0.0};
    int u = route[0];

    for (size_t i = 1; i < route.size(); i++) {
        int v = route[i];

        current_time += adj_matrix[u][v].second->avg_time;
        if(pickup_nodes.count(v)) {
            for(int delivery_node : pickup_nodes[v]) {
                delivery_nodes[delivery_node]++;
            }
            pickup_nodes.erase(v);
        } 

        if(delivery_nodes.count(v)) {
            sum_completion += current_time * delivery_nodes[v];
            delivery_nodes.erase(v);
            if (current_time > max_completion) {
                max_completion = current_time;
            }
        }

        u = v;
    }

    return {sum_completion, max_completion};

}


vector<vector<int>> Graph::kmeansPlusPlusClusters(
        int no_agents,
        unordered_map<int, pair<int,int>> orders)
{
    vector<int> order_ids;
    vector<pair<double,double>> centers;

    for (auto& kv : orders) {
        int oid = kv.first;
        int p   = kv.second.first;
        int d   = kv.second.second;

        double cx = (nodes[p]->lat + nodes[d]->lat) * 0.5;
        double cy = (nodes[p]->lon + nodes[d]->lon) * 0.5;

        order_ids.push_back(oid);
        centers.push_back({cx, cy});
    }

    int m = (int)centers.size();
    if (m == 0 || no_agents <= 0) {
        return {}; 
    }

    vector<vector<int>> clusters(no_agents);

    if (no_agents >= m) {
        int k = 0;
        for (int i = 0; i < m; ++i) {
            clusters[k].push_back(order_ids[i]);
            k = (k + 1) % no_agents;
        }
        return clusters;
    }

    int K = no_agents;
    vector<bool> used(m, false);
    vector<pair<double,double>> seeds;
    seeds.reserve(K);

    double meanx = 0.0, meany = 0.0;
    for (auto& c : centers) {
        meanx += c.first;
        meany += c.second;
    }
    meanx /= m;
    meany /= m;

    int first = -1;
    double best = 1e18;
    for (int i = 0; i < m; i++) {
        double dx = centers[i].first  - meanx;
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

    for (int k = 1; k < K; k++) {
        auto& last = seeds.back();

        for (int i = 0; i < m; i++) {
            if (!used[i]) {
                double dx = centers[i].first  - last.first;
                double dy = centers[i].second - last.second;
                double d2 = dx*dx + dy*dy;
                if (d2 < minDistSq[i]) minDistSq[i] = d2;
            }
        }

        int next = -1;
        double bestD = -1.0;
        for (int i = 0; i < m; i++) {
            if (!used[i] && minDistSq[i] > bestD) {
                bestD = minDistSq[i];
                next  = i;
            }
        }

        used[next] = true;
        seeds.push_back(centers[next]);
    }

    vector<int> capacity(K, m / K);
    int extra = m % K;
    for (int k = 0; k < extra; ++k) {
        capacity[k]++;         
    }

    vector<int> clusterSize(K, 0);

    for (int i = 0; i < m; i++) {
        double bestDist = 1e18;
        int bestCluster = -1;

        for (int k = 0; k < K; k++) {
            if (clusterSize[k] >= capacity[k]) continue; 

            double dx = centers[i].first  - seeds[k].first;
            double dy = centers[i].second - seeds[k].second;
            double d2 = dx*dx + dy*dy;

            if (d2 < bestDist) {
                bestDist    = d2;
                bestCluster = k;
            }
        }


        if (bestCluster == -1) {
            bestCluster = 0;
            for (int k = 1; k < K; k++) {
                if (clusterSize[k] < clusterSize[bestCluster])
                    bestCluster = k;
            }
        }

        clusters[bestCluster].push_back(order_ids[i]);
        clusterSize[bestCluster]++;
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
    unordered_set<int> pickups;
    unordered_set<int> deliveries;
    for (auto& kv : orders) {
        pickups.insert(kv.second.first);
        deliveries.insert(kv.second.second);
    }
    unordered_map<int,int> pos;
    for (size_t i = 0; i < route.size(); i++){
        if(pickups.count(route[i])){
            if(pos.find(route[i])==pos.end()){
                pos[route[i]]=i;
            }
        }
        else if(deliveries.count(route[i])){
            pos[route[i]] = i;
        }
    }

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
    output.reserve(no_agents);

    using Clock = chrono::high_resolution_clock;

    for (int k = 0; k < no_agents; ++k) {
        auto order_ids = clusters[k];

        if (order_ids.empty()) {
            output.push_back({{}, {}});
            continue;
        }

        double bestSum = 0;
        double bestMax = 0;
        vector<int> bestRoute = buildGreedyRoute(depot_node, orders,order_ids,bestSum,bestMax);

        auto res = computetime(bestRoute, orders, order_ids);
        bestSum = res.first;
        bestMax = res.second;
        
        double bestCost = bestSum;     

        auto start = Clock::now();

        while (true) {
            auto now = Clock::now();
            double elapsed = chrono::duration<double>(now - start).count();
            if (elapsed > 0.1) break;   

            vector<int> cand =
                (rand() % 2 ? twoPointSwap(bestRoute)
                            : shiftNode(bestRoute));

            if (!validPickupDelivery(cand, orders))
                continue;

            auto [sumC, maxC] = computetime(cand, orders, order_ids);
            double costC = sumC;

            if (costC < bestCost) {
                bestCost  = costC;
                bestRoute = std::move(cand);
            }
        }

        total_time += bestCost;
        output.push_back({ std::move(bestRoute), std::move(order_ids) });
    }

    return output;
}