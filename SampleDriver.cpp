#include<iostream>
#include "graph.hpp"
#include "json.hpp"
using namespace std;

int main(int argc, char* argv[]){
    if(argc<2){
        cerr<<"Usage:"<< argv[0] << " <graph.json>" << endl;
        return 1;
    }
    string filename = argv[0];
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }
    json j;
    fin>>j;
    auto meta=j["meta"];

    Graph g;
    for(auto &n: j["nodes"]){
        int id=n["id"];
        double lat=n["lat"];
        double lon=n["lon"];
        vector<string> pois;
        for(auto &p: n["pois"]) pois.push_back(p);
        g.addNode(id,lat,lon,pois);
    }
    for(auto &e: j["edges"]){
        int id=e["id"];
        int u=e["u"];
        int v=e["v"];
        double length=e["length"];
        double avg_time=e["average_time"];
        bool oneway=e["oneway"];
        string road_type=e["road_type"];
        vector<double> speed_profile;
         if (e.contains("speed_profile")) {
            for (auto &s : e["speed_profile"])
                speed_profile.push_back(s);
        }
        g.addEdge(id, u, v, length, avg_time, speed_profile, oneway, road_type);
    }

    
}