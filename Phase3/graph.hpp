#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <iostream>
#include <queue>
#include <chrono>
using namespace std;

// ---------- Existing classes ----------
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

    // ---------- Existing shortest path functions ----------
    pair<vector<int>,double> shortestPath_minDistance(int source, int destination,
                                                                vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool& possible);
    pair<vector<int>, double> shortestPath_minTime(int source, int destination,vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool &possible);
    pair<vector<int>, double> shortestPath_minTime_withSpeedProfile(int source, int destination,int start_time, vector<int> forbidden_nodes, vector<string> forbidden_road_types, bool &possible);
    
    vector<pair<double,int>> shortestPath_allDistances(int source);
    
    vector<int> knn(string poi_type,double query_lat,double query_lon,int K,string metric);
    
    vector<pair<vector<int>, double>> kShortestPaths_exact(int source, int target, int K) ;
    double approxShortestPath(int source, int destination, 
                                     double time_budget_ms, double acceptable_error_pct);

    bool isOverlapping(vector<int> path1, vector<int> path2, int threshold, int& overlap_count);
    
    vector<pair<vector<int>, double>> s_to_allnodes_shortestpaths(int source);
    vector<pair<vector<int>, double>> allnodes_to_t_shortest_paths(int target);

    vector<pair<vector<int>, double>> kShortestPaths_Heuristic(int source, int target, int K, int threshold);

    vector<pair<vector<int>, double>> kShortestPaths_Heuristic_svp(int source, int target, int K, int threshold);

    vector<vector<int>> nearestSeedClustering(int no_agents,unordered_map<int,pair<int,int>> orders);

   vector<int> buildGreedyRoute(int depot,unordered_map<int,pair<int,int>> orders,int no_agents,vector<int>cluster) ;
   pair<double,double> computetime(vector<int> route, unordered_map<int,pair<int,int>> orders);
   double getShortestPathTravelTime(int start, int end);
    vector<pair<vector<int>,vector<int>>> delivery_route(int no_agents, int depot_node, unordered_map<int,pair<int,int>> orders, double& total_time);

};
