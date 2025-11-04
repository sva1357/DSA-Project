#include<iostream>
#include<fstream>
#include<chrono>
#include <string>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace std;

#if defined(PHASE1)
    #include "graph.hpp"
    #include "QueryProcessor.hpp"
#elif defined(PHASE2)
    #include "graph.hpp"
    #include "QueryProcessor.hpp"
#else
    #error "No phase defined"
#endif

json processQuery(const json &q, Graph &g){
    string type=q["type"];

    #if defined(PHASE1)
        if(type=="shortest_path"){
            return shortest_path(q,g);
        }
        else if(type=="knn"){
            return knn(q,g);
        }
        else if(type=="remove_edge"){
            return removeEdge(q,g);
        }
        else if(type=="modify_edge"){
            return modify_edge(q,g);
        }
    #elif defined(PHASE2)
        if(type=="k_shortest_paths"){
            return k_shortest_paths(q,g);
        }
        else if(type=="k_shortest_paths_heuristic"){
            return k_shortest_paths_heuristic(q,g);
        }
        else if(type=="approx_shortest_path"){
            return approx_shortest_path(q,g);
        }
    #endif

    return json{{"error", "Unknown query type"}, {"query_type", type}};
}
int main(int argc, char* argv[]){
    if(argc!=4){
        cerr<<"Usage:"<< argv[0] << " <graph.json> <queries.json> <output.json>" << endl;
        return 1;
    }
    string filename = argv[1];
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
    string file1name = argv[2];
    ifstream fin(file1name);
    if (!fin.is_open()) {
        cerr << "Error: Could not open file " << file1name << endl;
        return 1;
    }
    json q;
    fin>>q;
    string file2name=argv[3];
    json out;
    out["meta"]=q["meta"];
    out["results"]=json::array();
    
    for(auto &u: q["events"]){
        auto start_time=chrono::high_resolution_clock::now();
        json result=processQuery(q,g);
        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        out["results"].push_back(result);
    }

    ofstream f(file2name);
    if(!f.is_open()){
        cerr<<"Can't write output file"<<endl;
        exit(1);
    }
    
}