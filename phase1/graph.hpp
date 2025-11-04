#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

class Node{

    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;

public:
    Node(int id, double lat, double lon, std::vector<std::string> pois)
        :id(id), lat(lat), lon(lon), pois(pois) {}

};

class Edge{

    int id;
    int u;
    int v;
    double len;
    double avg_time;
    std::vector<double> speed_profile;
    bool oneway;
    std::string  road_type;
    bool blocked;

public:
    Edge(int id, int u, int v, double length, double avg_time, std::vector<double> speed_profile, bool oneway, std::string road_type)
        :id(id), u(u), v(v), len(length), avg_time(avg_time), speed_profile(speed_profile), oneway(oneway), road_type(road_type), blocked(false) {}
};

class Graph{

    std::vector<Node*> nodes;
    std::unordered_map<int,Edge*> edges;
    std::vector<std::vector<std::pair<int,Edge*>>> adj; 
    int V;

public:
    Graph():V(0){}
    void addNode(int id, double lat, double lon, std::vector<std::string> pois){
        Node* n=new Node(id,lat,lon,pois);
        nodes.push_back(n);
        V++;
    }

    void addEdge(int id, int u, int v, double length, double avg_time, std::vector<double> speed_profile,
                    bool oneway, std::string road_type){
        Edge* e=new Edge(id,u,v,length,avg_time,speed_profile,oneway,road_type);
        edges[id]=e;
        if(adj.size()<V) adj.resize(V);
        adj[u].push_back({v,e});
        if(!oneway){
            adj[v].push_back({u,e});
        }
    }

    bool removeEdge(int id){
        if(edges.find(id)==edges.end()) return false;
        Edge* e=edges[id];
        e->blocked=true;
        return true;
    }

    void modifyEdge(int id, bool change_time, double new_avg_time, bool change_speed_profile, 
                        std::vector<double> new_speed_profile, bool change_length, double new_length){
        if(edges.find(id)==edges.end()) return;
        Edge* e=edges[id];
        if(change_time) e->avg_time=new_avg_time;
        if(change_speed_profile) e->speed_profile=new_speed_profile;
        if(change_length) e->len=new_length;
        e->blocked=false;
    }

    double euclideanDistance(int u, int v){
        double lat1 = nodes[u]->lat;
        double lon1 = nodes[u]->lon;
        double lat2 = nodes[v]->lat;
        double lon2 = nodes[v]->lon;
        return std::sqrt((lat1 - lat2) * (lat1 - lat2) + (lon1 - lon2) * (lon1 - lon2));
    }
    
    std::pair<std::vector<int>,int> shortestPath_minDistance(int source, int destination, std::vector<int> forbidden_nodes, std::vector<string> forbidden_road_types, bool possible){
        
        std::vector<double> dist(V, std::numeric_limits<double>::max());
        std::vector<int> prev(V, -1);
        std::vector<bool> visited(V, false);

        std::unordered_map<int, bool> forbidden_node_map;
        for (int fn : forbidden_nodes) forbidden_node_map[fn] = true;

        std::unordered_map<std::string, bool> forbidden_road_type_map;
        for (const std::string& frt : forbidden_road_types) forbidden_road_type_map[frt] = true;

        dist[source] = 0.0;
        using PDI = std::pair<double, int>;
        std::priority_queue<PDI, std::vector<PDI>, std::greater<PDI>> pq;
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

        std::vector<int> path;
        if (dist[destination] == std::numeric_limits<double>::max()) {
            possible = false;
            return {{}, 0};
        }
        for (int at = destination; at != -1; at = prev[at])
            path.push_back(at);
        std::reverse(path.begin(), path.end());
        possible = true;
        return {path, static_cast<int>(dist[destination])};
        
    }

};



