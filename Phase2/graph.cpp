#include "graph.hpp"
#include <unordered_set>

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

// pair<vector<int>, double> Graph::shortestPath_minTime_withSpeedProfile(
//     int source, int destination,
//     int start_time, vector<int> forbidden_nodes,
//     vector<string> forbidden_road_types, bool &possible)
// {
//     vector<double> dist(V, numeric_limits<double>::max());
//     vector<int> prev(V, -1);

//     unordered_map<int, bool> forbidden_node_map;
//     for(int fn: forbidden_nodes) forbidden_node_map[fn] = true;

//     unordered_map<string, bool> forbidden_road_map;
//     for(const string &frt: forbidden_road_types) forbidden_road_map[frt] = true;

//     if(forbidden_node_map.count(source) || forbidden_node_map.count(destination)){
//         possible = false;
//         return {{}, -1};
//     }

//     dist[source] = start_time;
//     using PDI = pair<double,int>;
//     priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
//     pq.push({start_time, source});

//     while(!pq.empty()){
//         auto [curr_time, u] = pq.top(); pq.pop();

//         if(curr_time > dist[u]) continue;  //  ignore outdated queue entries

//         if(u == destination) break;

//         for(const auto &[v, e] : adj[u]){

//             if(forbidden_node_map.count(v)) continue;
//             if(e->blocked) continue;
//             if(forbidden_road_map.count(e->road_type)) continue;
//             if(e->speed_profile.empty()) continue; //  avoid mod 0

//             int t_index = (int(curr_time) % e->speed_profile.size());
//             double speed = e->speed_profile[t_index];
//             if(speed <= 0) continue;

//             double travel_time = e->len / speed;
//             double new_time = curr_time + travel_time;

//             if(new_time < dist[v]){
//                 dist[v] = new_time;
//                 prev[v] = u;
//                 pq.push({new_time, v});
//             }
//         }
//     }

//     if(dist[destination] == numeric_limits<double>::max()){
//         possible = false;
//         return {{}, -1};
//     }

//     vector<int> path;
//     for(int at = destination; at != -1; at = prev[at]) path.push_back(at);
//     reverse(path.begin(), path.end());

//     possible = true;
//     return {path, dist[destination] - start_time}; // spent time
// }

vector<pair<double,int>> Graph::shortestPath_allDistances(int source){
    vector<double> dist(V, numeric_limits<double>::max());
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
            double weight = e->len;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    vector<pair<double,int>> allDistances;
    for (int i = 0; i < V; ++i) {
        allDistances.push_back({dist[i], i});
    }
    return allDistances;
}

vector<int> Graph::knn(string poi_type,double query_lat,double query_lon,int K,string metric){

    double mindist = numeric_limits<double>::max();
    int node = -1;
    vector<pair<double, int>> euclideanDistances;


    for (int i = 0;i < V;i++) {
        bool has_poi = false;
        for (auto& poi : nodes[i]->pois) {
            if (poi == poi_type) { has_poi = true; break; }
        }

        double d = sqrt(
            (nodes[i]->lat - query_lat) * (nodes[i]->lat - query_lat) +
            (nodes[i]->lon - query_lon) * (nodes[i]->lon - query_lon)
        );

        if (d < mindist) {
            mindist = d;
            node = i;
        }

        if (has_poi) {
            euclideanDistances.push_back({d, i});
        }
    }

    if(metric=="euclidean"){
        sort(euclideanDistances.begin(), euclideanDistances.end());
        vector<int> knearest;
        K=min(K, (int)euclideanDistances.size());
        for (int i = 0; i <K; ++i) knearest.push_back(euclideanDistances[i].second);
        return knearest;
    }

    vector<pair<double, int>> allDistances = shortestPath_allDistances(node);
    vector<pair<double, int>> pathDistances;
    for (auto& [d, i] : allDistances) {
        bool has_poi = false;
        for (auto& poi : nodes[i]->pois) {
            if (poi == poi_type) { has_poi = true; break; }
        }
        if (has_poi) {
            pathDistances.push_back({d, i});
        }
    }

    sort(pathDistances.begin(), pathDistances.end());
    vector<int> knearest;
    K=min(K, (int)pathDistances.size());
    for (int i = 0; i <K; ++i) knearest.push_back(pathDistances[i].second);
    return knearest;
    
}

vector<pair<vector<int>, double>> Graph::kShortestPaths_exact(int source, int target, int K) {
   

    bool possible;
    vector<pair<vector<int>, double>> result;
    vector<pair<vector<int>, double>> candidates;

    // FIRST SHORTEST PATH
    auto first = shortestPath_minDistance(source, target, {}, {}, possible);
    if (!possible || first.first.empty()) return {};
    result.push_back(first);

    for (int k = 1; k < K; ++k) {

        const auto &prevPath = result.back().first;


        for (size_t i = 0; i + 1 < prevPath.size(); ++i) {

            int spurNode = prevPath[i];
            vector<int> root_path(prevPath.begin(), prevPath.begin() + i + 1);

           
            unordered_set<int> blocked_nodes;
            for (int node : root_path)
                if (node != spurNode)
                    blocked_nodes.insert(node);

            
            vector<Edge*> blocked_edges;

            for (const auto &rp : result) {
                if (rp.first.size() <= i + 1) continue;

                bool samePrefix = true;
                for (size_t t = 0; t <= i; ++t)
                    if (rp.first[t] != root_path[t]) {
                        samePrefix = false;
                        break;
                    }

                if (!samePrefix) continue;

                int from = rp.first[i];
                int to   = rp.first[i + 1];

                for (auto &[nbr, e] : adj[from]) {
                    if (nbr == to) {
                        e->blocked = true;
                        blocked_edges.push_back(e);
                    }
                }
            }

            bool spurPossible;
            vector<int> forbidden_nodes_vec(blocked_nodes.begin(), blocked_nodes.end());

            auto spur = shortestPath_minDistance(
                spurNode, target,
                forbidden_nodes_vec,
                {},
                spurPossible
            );

            for (Edge *e : blocked_edges) e->blocked = false;

            if (!spurPossible || spur.first.empty()) continue;

            vector<int> total_path = root_path;
            total_path.pop_back();
            total_path.insert(total_path.end(),
                              spur.first.begin(), spur.first.end());

        
          unordered_set<int> seen;
            bool loopless = true;
            for (int n : total_path)
                if (!seen.insert(n).second) {
                    loopless = false;
                    break;
                }
            if (!loopless) continue;

    
            double cost = 0;
            bool ok = true;

            for (size_t j = 0; j + 1 < total_path.size(); ++j) {
                int u = total_path[j];
                int v = total_path[j + 1];
                bool found = false;

                for (auto &[nbr, e] : adj[u]) {
                    if (nbr == v) {
                        cost += e->len;
                        found = true;
                        break;
                    }
                }
                if (!found) { ok = false; break; }
            }
            if (!ok) continue;

        
            bool exists = false;
            for (auto &c : candidates)
                if (c.first == total_path)
                    exists = true;

            if (!exists)
                candidates.push_back({total_path, cost});
        }

        if (candidates.empty()) break;

        sort(candidates.begin(), candidates.end(),
             [](const auto &a, const auto &b) {
                 return a.second < b.second;
             });

        result.push_back(candidates.front());

        candidates.erase(candidates.begin()); 
    }

    return result;
}


// double Graph::approxShortestDistance(int source, int destination, 
//                                      double time_budget_ms, double acceptable_error_pct)
// {
//     const double INF = numeric_limits<double>::infinity();
//     vector<double> dist(V, INF);

//     priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> pq;

//     dist[source] = 0.0;
//     pq.push({0.0, source});

//     auto start_time = chrono::high_resolution_clock::now();

//     while(!pq.empty()) {
//         auto now = chrono::high_resolution_clock::now();
//         double elapsed_ms = chrono::duration<double, milli>(now - start_time).count();

//         // Time cutoff: return best known so far
//         if(elapsed_ms > time_budget_ms) {
//             return dist[destination] * (1.0 + acceptable_error_pct / 100.0);
//         }

//         auto [d, u] = pq.top();
//         pq.pop();

//         if(d > dist[u]) continue;
//         if(u == destination) break;

//         for(auto &nbr : adj[u]) {
//             int v = nbr.first;
//             Edge* e = nbr.second;
//             if(e->blocked) continue;

//             double w = e->len; 

//             if(dist[u] + w < dist[v]) {
//                 dist[v] = dist[u] + w;
//                 pq.push({dist[v], v});
//             }
//         }
//     }

//     if(dist[destination] == INF) return -1.0;

//     return dist[destination] * (1.0 + acceptable_error_pct / 100.0);
// }

double Graph::approxShortestPath(
    int source,
    int destination,
    double time_budget_ms,
    double acceptable_error_pct)
{
    using Clock = std::chrono::high_resolution_clock;
    auto start_time = Clock::now();

    if (source == destination) return 0.0;

    const double INF = std::numeric_limits<double>::infinity();
    vector<double> dist(V, INF);
    dist[source] = 0.0;

    // Precompute heuristic (straight-line distances)
    vector<double> h_val(V);
    for (int i = 0; i < V; i++) {
        h_val[i] = euclideanDistance(i, destination);
    }

    using QItem = pair<double, int>; // (f = g + h, node)
    priority_queue<QItem, vector<QItem>, greater<QItem>> pq;
    pq.push({h_val[source], source});

    double best_path = INF;
    int iteration = 0;
    const int CHECK_INTERVAL = 100;

    while (!pq.empty()) {
        iteration++;

        // Check time constraint periodically
        if (iteration % CHECK_INTERVAL == 0) {
            double elapsed = std::chrono::duration<double, std::milli>(
                Clock::now() - start_time).count();
            if (elapsed > time_budget_ms) {
                break;
            }
        }

        auto [f, u] = pq.top();
        pq.pop();

        // If not best known, skip
        if (f > dist[u] + h_val[u] + 1e-12) {
            continue;
        }

        // If reached destination
        if (u == destination) {
            best_path = dist[u];

            if (!pq.empty()) {
                double next_best_f = pq.top().first;

                // Approximation condition:
                // If the next best possible path can't beat current path
                if (next_best_f >= best_path * (1.0 + acceptable_error_pct / 100.0)) {
                    break; // Good enough!
                }
            }

            continue; // Keep searching for possibly better path
        }

        // Expand neighbors
        for (auto &nbr : adj[u]) {
            int v = nbr.first;
            double w = nbr.second->len;

            double new_g = dist[u] + w;
            if (new_g < dist[v]) {
                dist[v] = new_g;
                pq.push({new_g + h_val[v], v});
            }
        }
    }

    return (best_path < INF) ? best_path : -1.0;
}

bool Graph::isOverlapping(vector<int> path1, vector<int> path2, int threshold, int& overlap_count){
    overlap_count=0;
    unordered_map<int,int> edges_map;
    for(size_t i=0; i+1<path1.size(); i++){
        edges_map[path1[i]]=path1[i+1];
    }
    for(size_t i=0; i+1<path2.size(); i++){
        int u=path2[i];
        int v=path2[i+1];
        if(edges_map.find(u)!=edges_map.end() && edges_map[u]==v){
            overlap_count++;
        }
    }
    double overlap_percentage=(overlap_count*100)/min(path1.size()-1,path2.size()-1);
    if(overlap_percentage>=threshold) return true;
    return false;
}  

vector<pair<vector<int>, double>> Graph::s_to_allnodes_shortestpaths(int source){
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
            double weight = e->len;
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

vector<pair<vector<int>,double>> Graph::allnodes_to_t_shortest_paths(int target){
    vector<pair<vector<int>,double>> result;
    result.resize(V);

    vector<vector<pair<int, Edge*>>> adj_rev(V);
    for (int u = 0; u < V; ++u) {
        for (const auto &pe : adj[u]) {
            int v = pe.first;
            Edge* e = pe.second;
            adj_rev[v].push_back({u, e});
        }
    }

    vector<double> dist(V, numeric_limits<double>::max());
    vector<int> prev(V, -1);
    vector<bool> visited(V, false);

    using PDI = pair<double, int>;
    priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
    dist[target] = 0.0;
    pq.push({0.0, target});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (visited[u]) continue;
        visited[u] = true;

        for (const auto& pr : adj_rev[u]) {
            int v = pr.first;
            Edge* e = pr.second;
            if (visited[v]) continue;
            if (e->blocked) continue;
            double weight = e->len;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    for (int source = 0; source < V; ++source) {
        vector<int> path;
        if (dist[source] == numeric_limits<double>::max()) {
            result[source] = {path, -1};
            continue;
        }
        for (int at = source; at != -1; at = prev[at]) path.push_back(at);
        result[source] = {path, dist[source]};
    }
    return result;
}

vector<pair<vector<int>, double>> Graph::kShortestPaths_Heuristic_svp(int source, int target, int K, int threshold){
    vector<pair<vector<int>,double>> result;
    vector<pair<vector<int>, double>> all_paths=s_to_allnodes_shortestpaths(source);
    if(all_paths[target].first.empty()) return result;
    if(V<450&&edges.size()<750){
        result=kShortestPaths_Heuristic(source, target, K, threshold);
    }
    else{
        result.push_back(all_paths[target]);
    }
    
    vector<pair<vector<int>, double>> to_t_paths=allnodes_to_t_shortest_paths(target);

    unordered_map<int, bool> in_result;
    for(int i=0; i<V; i++) in_result[i]=false;
    for(size_t i=0; i<result[0].first.size(); i++) in_result[result[0].first[i]]=true;
    vector<pair<double,int>> candidates;
    for(int i=0; i<V; i++){
        if(all_paths[i].first.empty()) continue;
        if(to_t_paths[i].first.empty()) continue;
        if(in_result[i]) continue;
        if(i==source || i==target) continue;
        candidates.push_back({all_paths[i].second + to_t_paths[i].second, i});
    }
    sort(candidates.begin(), candidates.end());

    for(int theta=threshold; theta<=80; theta+=min(5,theta/5)){

        for(size_t i=0;i<candidates.size() && result.size()<(size_t)K;i++){
            auto curr=candidates[i];
            if(in_result.find(curr.second)!=in_result.end() && in_result[curr.second]) continue;
            if(i==0) continue; // already added shortest path
            if(curr.second==source || curr.second==target) continue;
            int intermediate=curr.second;
            vector<int> path;
            auto& path1=all_paths[intermediate].first;
            auto& path2=to_t_paths[intermediate].first;
            for(size_t i=0; i<path1.size(); i++) path.push_back(path1[i]);
            for(size_t i=1; i<path2.size(); i++) path.push_back(path2[i]);
            double total_cost=curr.first;

            bool is_overlapping=false;
            for(size_t i=0; i<result.size(); i++){
                int overlap_count=0;
                
                if(isOverlapping(path, result[i].first, theta, overlap_count)){
                    is_overlapping=true;
                    break;
                }

            }
            
            if(is_overlapping) continue;
            in_result[intermediate]=true;
            result.push_back({path, total_cost});
        }
        if(result.size()>=(size_t)K) break;
    }

    if(result.size()<(size_t)K){
        for(size_t i=0; i<candidates.size() && result.size()<(size_t)K; i++){
            int intermediate=candidates[i].second;
            vector<int> path;
            auto& path1=all_paths[intermediate].first;
            auto& path2=to_t_paths[intermediate].first;
            for(size_t j=0; j<path1.size(); j++) path.push_back(path1[j]);
            for(size_t j=1; j<path2.size(); j++) path.push_back(path2[j]);
            double total_cost=candidates[i].first;
            result.push_back({path, total_cost});
        }
    }
    return result;
}

struct Label{
    int node;
    double cost;
    int parent;
    vector<int> overlap_counts;
    bool removed = false;
};

struct PQEntry{
    double cost;
    int label_id;
    bool operator>(const PQEntry& other) const {
        return cost > other.cost;
    }
};

vector<pair<vector<int>, double>> Graph::kShortestPaths_Heuristic(int source, int target, int K, int threshold){
    vector<pair<vector<int>,double>> result;
    bool possible;
    auto first_path=shortestPath_minDistance(source,target,{}, {}, possible);
    if(!possible || first_path.first.empty()) return result;
    result.push_back(first_path);
    bool found_new_path= true;
    vector<unordered_map<int,int>> result_edge_maps;
    vector<int> result_path_lengths;
    for(auto& p: result){
        unordered_map<int,int> edge_map;
        for(size_t i=0; i+1<p.first.size(); i++){
            edge_map[p.first[i]]=p.first[i+1];
        }
        result_edge_maps.push_back(edge_map);
        result_path_lengths.push_back((int)p.first.size()-1);
    }
    while(result.size()<(size_t)K && found_new_path){
        found_new_path=false;
        priority_queue<PQEntry, vector<PQEntry>, greater<PQEntry>> pq;
        vector<Label*> labels;
        vector<vector<int>> lambda(V);
        Label* first_label=new Label{source,0.0,-1,{}};
        for(size_t i=0; i<result.size(); i++){
            first_label->overlap_counts.push_back(0);
        }
        labels.push_back(first_label);
        lambda[source].push_back(0);
        pq.push({0.0,0});
        while(!pq.empty()){
            auto curr=pq.top();
            pq.pop();

            if(labels[curr.label_id]->removed) continue;

            Label* curr_label=labels[curr.label_id];

            if(curr_label->node==target){
                vector<int> path;
                int label_index=curr.label_id;
                while(label_index!=-1){
                    Label* l=labels[label_index];
                    path.push_back(l->node);
                    label_index=l->parent;
                }
                reverse(path.begin(), path.end());
                double total_cost=curr_label->cost;
                found_new_path=true;
                result.push_back({path,total_cost});
                break;
            }

            for(auto& [nbr,e]: adj[curr_label->node]){
                if(nbr==source) continue;
                
                double new_cost=curr_label->cost + e->len;
                
                Label* new_label=new Label{nbr,new_cost,curr.label_id,{}};

                bool is_overlapping=false;
                for(size_t i=0; i<result.size(); i++){
                    int overlap_count=curr_label->overlap_counts[i];
                    if(result_edge_maps[i].find(curr_label->node)!=result_edge_maps[i].end() &&
                       result_edge_maps[i][curr_label->node]==nbr){
                        overlap_count++;
                    }
                    new_label->overlap_counts.push_back(overlap_count);
                    int overlap_percentage=(overlap_count*100)/(result_path_lengths[i]);
                    if(overlap_percentage>threshold){
                        is_overlapping=true;
                        break;
                    }
                }
                if(is_overlapping) {
                    delete new_label;
                    continue;
                }

                bool dominated=false;

                for(int label_id: lambda[nbr]){
                    if(labels[label_id]->cost <= new_cost){
                        bool found_better=true;
                        for(size_t i=0; i<new_label->overlap_counts.size(); i++){
                            if(new_label->overlap_counts[i] < labels[label_id]->overlap_counts[i]){
                                found_better=false;
                                break;
                            }
                        }
                        if(found_better){
                            dominated=true;
                            break;
                        }
                    }
                        
                }
                if(dominated) {
                    delete new_label;
                    continue;
                }

                vector<int> to_remove;
                for(size_t i=0; i<lambda[nbr].size(); i++){
                    int label_id=lambda[nbr][i];
                    if(labels[label_id]->cost >= new_cost){
                        bool found_worse=true;
                        for(size_t j=0; j<new_label->overlap_counts.size(); j++){
                            if(new_label->overlap_counts[j] > labels[label_id]->overlap_counts[j]){
                                found_worse=false;
                                break;
                            }
                        }
                        if(found_worse){
                            to_remove.push_back(i);
                        }
                    }
                }
                for(int i=to_remove.size()-1; i>=0; i--){
                    labels[lambda[nbr][to_remove[i]]]->removed=true;
                    lambda[nbr].erase(lambda[nbr].begin()+to_remove[i]);
                }
                labels.push_back(new_label);
                lambda[nbr].push_back((int)labels.size()-1);
                pq.push({new_cost, (int)labels.size()-1});
            }
            labels[curr.label_id]->removed=true;
        }
        for(auto l: labels) delete l;
        if(found_new_path){
            unordered_map<int,int> edge_map;
            auto& p=result.back();
            for(size_t i=0; i+1<p.first.size(); i++){
                edge_map[p.first[i]]=p.first[i+1];
            }
            result_edge_maps.push_back(edge_map);
            result_path_lengths.push_back((int)p.first.size());
        }

    }
    
    return result;
}