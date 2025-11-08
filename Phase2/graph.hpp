#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <queue>
using namespace std;

class Node{
public:
    int id;
    double lat;
    double lon;
    vector<string> pois;

    Node(int id, double lat, double lon, vector<string> pois)
        :id(id), lat(lat), lon(lon), pois(pois) {}
};

class Edge{
public:
    int id;
    int u;
    int v;
    double len;
    double avg_time;
    vector<double> speed_profile;
    bool oneway;
    string  road_type;
    bool blocked;

    Edge(int id, int u, int v, double length, double avg_time, vector<double> speed_profile, bool oneway, string road_type)
        :id(id), u(u), v(v), len(length), avg_time(avg_time), speed_profile(speed_profile), oneway(oneway), road_type(road_type), blocked(false) {}
};

class Graph{

    vector<Node*> nodes;
    unordered_map<int,Edge*> edges;
    vector<vector<pair<int,Edge*>>> adj; 
    int V;

public:
    Graph();
    void addNode(int id, double lat, double lon, vector<string> pois);

    void addEdge(int id, int u, int v, double length, double avg_time, vector<double> speed_profile,
                    bool oneway, string road_type);

    bool removeEdge(int id);

    void modifyEdge(int id, bool change_time, double new_avg_time, bool change_speed_profile, 
                        vector<double> new_speed_profile, bool change_length, double new_length, bool change_road_type, string new_road_type);

    double euclideanDistance(int u, int v);
    
    pair<vector<int>,double> shortestPath_minDistance(int source, int destination,
                                                                vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool& possible);
    pair<vector<int>, double> shortestPath_minTime(int source, int destination,vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool &possible);

    pair<vector<int>, double> shortestPath_minTime_withSpeedProfile(int source, int destination,int start_time, vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool &possible);
    
    vector<pair<double,int>> shortestPath_allDistances(int source);
    
    vector<int> knn(string poi_type,double query_lat,double query_lon,int K,string metric);
    vector<pair<vector<int>, double>> kShortestPaths_exact(int source, int target, int K) ;
        
};



