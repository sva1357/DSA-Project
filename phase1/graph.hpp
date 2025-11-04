#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <queue>

class Node{
public:
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;

    Node(int id, double lat, double lon, std::vector<std::string> pois)
        :id(id), lat(lat), lon(lon), pois(pois) {}
};

class Edge{
public:
    int id;
    int u;
    int v;
    double len;
    double avg_time;
    std::vector<double> speed_profile;
    bool oneway;
    std::string  road_type;
    bool blocked;

    Edge(int id, int u, int v, double length, double avg_time, std::vector<double> speed_profile, bool oneway, std::string road_type)
        :id(id), u(u), v(v), len(length), avg_time(avg_time), speed_profile(speed_profile), oneway(oneway), road_type(road_type), blocked(false) {}
};

class Graph{

    std::vector<Node*> nodes;
    std::unordered_map<int,Edge*> edges;
    std::vector<std::vector<std::pair<int,Edge*>>> adj; 
    int V;

public:
    Graph();
    void addNode(int id, double lat, double lon, std::vector<std::string> pois);

    void addEdge(int id, int u, int v, double length, double avg_time, std::vector<double> speed_profile,
                    bool oneway, std::string road_type);

    bool removeEdge(int id);

    void modifyEdge(int id, bool change_time, double new_avg_time, bool change_speed_profile, 
                        std::vector<double> new_speed_profile, bool change_length, double new_length);

    double euclideanDistance(int u, int v);
    
    std::pair<std::vector<int>,double> shortestPath_minDistance(int source, int destination,
                                                                std::vector<int> forbidden_nodes, std::vector<std::string> forbidden_road_types, bool& possible);
        
};



