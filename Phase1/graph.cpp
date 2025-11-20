#include "graph.hpp"

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
// pair<vector<int>, double> Graph::shortestPath_minTime(int source, int destination,
//                     vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool &possible){

//     vector<double> dist(V, numeric_limits<double>::max());
//     vector<int> prev(V, -1);
//     vector<bool> visited(V, false);

//     unordered_map<int, bool> forbidden_node_map;
//     for(int fn: forbidden_nodes) forbidden_node_map[fn] = true;

//     unordered_map<string, bool> forbidden_road_map;
//     for(const string& frt: forbidden_road_types) forbidden_road_map[frt] = true;

//     if(forbidden_node_map.count(source) || forbidden_node_map.count(destination)){
//         possible = false;
//         return {{}, -1};
//     }

//     dist[source] = 0.0;
//     using PDI = pair<double,int>;
//     priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
//     pq.push({0.0, source});

//     while(!pq.empty()){
//         int u = pq.top().second; pq.pop();
//         if(visited[u] || forbidden_node_map.count(u)) continue;
//         visited[u] = true;

//         if(u == destination) break;

//         for(const auto& [v, e] : adj[u]){
//             if(visited[v] || forbidden_node_map.count(v)) continue;
//             if(e->blocked) continue;
//             if(forbidden_road_map.count(e->road_type)) continue;

//             double weight = e->avg_time;
//             if(dist[u] + weight < dist[v]){
//                 dist[v] = dist[u] + weight;
//                 prev[v] = u;
//                 pq.push({dist[v], v});
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
//     return {path, dist[destination]};
// }

pair<vector<int>, double> Graph::shortestPath_minTime_withSpeedProfile(
    int source, int destination,
    vector<int> forbidden_nodes,
    vector<string> forbidden_road_types, bool &possible)
{
    int start_time = 0;
    vector<double> dist(V, numeric_limits<double>::max());
    vector<int> prev(V, -1);

    // Map for quick lookup of forbidden nodes
    unordered_map<int, bool> forbidden_node_map;
    for(int fn: forbidden_nodes) forbidden_node_map[fn] = true;

    // Map for quick lookup of forbidden road types
    unordered_map<string, bool> forbidden_road_map;
    for(const string &frt: forbidden_road_types) forbidden_road_map[frt] = true;

    // Check if source or destination is forbidden
    if(forbidden_node_map.count(source) || forbidden_node_map.count(destination)){
        possible = false;
        return {{}, -1};
    }

    dist[source] = start_time;
    using PDI = pair<double,int>;
    priority_queue<PDI, vector<PDI>, greater<PDI>> pq;
    pq.push({start_time, source});

    while(!pq.empty()){
        auto [curr_time, u] = pq.top(); pq.pop();

        if(curr_time > dist[u]) continue;  // ignore outdated entries
        if(u == destination) break;

        for(const auto &[v, e] : adj[u]){

            if(forbidden_node_map.count(v)) continue;
            if(e->blocked) continue;
            if(forbidden_road_map.count(e->road_type)) continue;

            double new_time;

            if(e->speed_profile.empty()){
                // If no speed profile, use average travel time
                new_time = curr_time + e->avg_time;
            }
            else{
                // Travel edge accounting for speed changes every 15 minutes
                double remaining_len = e->len;
                double t = curr_time;  // current time on edge

                while(remaining_len > 1e-9){
                    int t_index = std::min(int(t / 900.0), (int)e->speed_profile.size() - 1);
                    double speed = e->speed_profile[t_index];
                    if(speed <= 0) break;  // cannot travel

                    double interval_end = (t_index + 1) * 900.0;  // end of current 15-min interval
                    double dt = interval_end - t;  // time left in this interval
                    double d_can_travel = speed * dt;

                    if(d_can_travel >= remaining_len){
                        t += remaining_len / speed;
                        remaining_len = 0;
                    } else {
                        remaining_len -= d_can_travel;
                        t = interval_end;
                    }
                }

                new_time = t;
            }

            if(new_time < dist[v]){
                dist[v] = new_time;
                prev[v] = u;
                pq.push({new_time, v});
            }
        }
    }

    if(dist[destination] == numeric_limits<double>::max()){
        possible = false;
        return {{}, -1};
    }

    // Reconstruct path
    vector<int> path;
    for(int at = destination; at != -1; at = prev[at]) path.push_back(at);
    std::reverse(path.begin(), path.end());

    possible = true;
    return {path, dist[destination] - start_time}; // spent time
}

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
        if(dist[i] == numeric_limits<double>::max()) continue;
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

void Graph::printGraph(){
    cout << "Graph Nodes:" << endl;
    for (const auto& node : nodes) {
        cout << "Node ID: " << node->id << ", Lat: " << node->lat << ", Lon: " << node->lon << ", POIs: ";
        for (const auto& poi : node->pois) {
            cout << poi << " ";
        }
        cout << endl;
    }

    cout << "Graph Edges:" << endl;
    for (const auto& [id, edge] : edges) {
        cout << "Edge ID: " << edge->id << ", From: " << edge->u << ", To: " << edge->v
             << ", Length: " << edge->len << ", Avg Time: " << edge->avg_time
             << ", Oneway: " << (edge->oneway ? "Yes" : "No") << ", Road Type: " << edge->road_type
             << ", Blocked: " << (edge->blocked ? "Yes" : "No") << endl;
    }
}



