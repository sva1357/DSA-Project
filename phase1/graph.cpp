#include "graph.hpp"

Graph::Graph():V(0){}

void Graph::addNode(int id, double lat, double lon, vector<string> pois){
    Node* n=new Node(id,lat,lon,pois);
    nodes.push_back(n);
    V++;
}

void Graph::addEdge(int id, int u, int v, double length, double avg_time, vector<double> speed_profile,
                bool oneway, string road_type){
    Edge* e=new Edge(id,u,v,length,avg_time,speed_profile,oneway,road_type);
    edges[id]=e;
    if(adj.size()<V) adj.resize(V);
    adj[u].push_back({v,e});
    if(!oneway){
        adj[v].push_back({u,e});
    }
}

bool Graph::removeEdge(int id){
    if(edges.find(id)==edges.end()) return false;
    Edge* e=edges[id];
    e->blocked=true;
    return true;
}

void Graph::modifyEdge(int id, bool change_time, double new_avg_time, bool change_speed_profile, 
                    vector<double> new_speed_profile, bool change_length, double new_length){
    if(edges.find(id)==edges.end()) return;
    Edge* e=edges[id];
    if(change_time) e->avg_time=new_avg_time;
    if(change_speed_profile) e->speed_profile=new_speed_profile;
    if(change_length) e->len=new_length;
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
            double weight = euclideanDistance(u, v);
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
            double weight = euclideanDistance(u, v);
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

vector<pair<vector<int>, double>> Graph::kShortestPaths_exact(int source, int target, int K, string mode) {
    bool possible;
    vector<int> forbidden_nodes;
    vector<string> forbidden_road_types;

    
    pair<vector<int>, double> firstPath;
    if (mode == "distance")
        firstPath = shortestPath_minDistance(source, target, forbidden_nodes, forbidden_road_types, possible);
    else
        firstPath = shortestPath_minTime(source, target, forbidden_nodes, forbidden_road_types, possible);

    if (!possible)
        return {}; 

    vector<pair<vector<int>, double>> A; 
    A.push_back(firstPath);

    using Candidate = pair<double, vector<int>>;
    priority_queue<Candidate, vector<Candidate>, greater<Candidate>> B; 

   
    for (int k = 1; k < K; ++k) {
        const vector<int>& prevPath = A[k - 1].first;

       
        for (size_t i = 0; i + 1 < prevPath.size(); ++i) {
            int spurNode = prevPath[i];
            vector<int> rootPath(prevPath.begin(), prevPath.begin() + i + 1);

           
            vector<pair<int, pair<int, Edge*>>> removedEdges;
            for (const auto& [path, _] : A) {
                if (path.size() > i && equal(rootPath.begin(), rootPath.end(), path.begin())) {
                    int u = path[i];
                    int v = path[i + 1];
                    auto& adjList = adj[u];
                    auto it = remove_if(adjList.begin(), adjList.end(),
                        [v](auto& pr) { return pr.first == v; });
                    if (it != adjList.end())
                        adjList.erase(it, adjList.end());
                }
            }

            vector<int> blockedNodes(rootPath.begin(), rootPath.end() - 1);

         
            bool spurPossible;
            pair<vector<int>, double> spurPath;
            if (mode == "distance")
                spurPath = shortestPath_minDistance(spurNode, target, blockedNodes, {}, spurPossible);
            else
                spurPath = shortestPath_minTime(spurNode, target, blockedNodes, {}, spurPossible);

            if (spurPossible && !spurPath.first.empty()) {
                vector<int> totalPath = rootPath;
                totalPath.pop_back();
                totalPath.insert(totalPath.end(), spurPath.first.begin(), spurPath.first.end());

                // Compute total length/time
                double totalDist = 0.0;
                if (mode == "distance") {
                    for (size_t j = 0; j + 1 < totalPath.size(); ++j)
                        totalDist += euclideanDistance(totalPath[j], totalPath[j + 1]);
                } else {
                    for (size_t j = 0; j + 1 < totalPath.size() - 1; ++j) {
                        int u = totalPath[j];
                        int v = totalPath[j + 1];
                        for (auto& [to, e] : adj[u]) {
                            if (to == v) { totalDist += e->avg_time; break; }
                        }
                    }
                }

                B.push({totalDist, totalPath});
            }
        }

        // Step 4: If no candidates remain, stop
        if (B.empty()) break;

        // Step 5: Pick next shortest path
        auto nextPath = B.top(); B.pop();
        A.push_back({nextPath.second, nextPath.first});
    }

    return A;
}

