#include<iostream>
#include<fstream>
#include<chrono>
#include <string>
#include <nlohmann/json.hpp>
#include "algorithm"
using json = nlohmann::json;
using namespace std;

#if defined(PHASE1)
    #include "graph.hpp"
    #include "QueryProcessor.hpp"
#elif defined(PHASE2)
    #include "graph.hpp"
    #include "QueryProcessor.hpp"
#elif defined(PHASE3)
    #include "graph.hpp"
    #include "QueryProcessor.hpp"
#else
    #error "No phase defined"
#endif

json processQuery(const json &q, Graph &g){

    #if defined(PHASE3)
        return delivery_route_optimization(q,g);
    #else
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
    #endif

    
}
int main(int argc, char* argv[]){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout.tie(nullptr);

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
    string buffer((istreambuf_iterator<char>(fin)), istreambuf_iterator<char>());
    json j = json::parse(buffer, nullptr, false);
    if(j.is_discarded()){ cerr << "Invalid JSON file\n"; return 1; }
    auto meta=j["meta"];

    Graph g;
    int V=0;
    for(auto &n: j["nodes"]){
        int id=n["id"];
        double lat=n["lat"];
        double lon=n["lon"];
        vector<string> pois;
        for(auto &p: n["pois"]) pois.push_back(p);
        g.addNode(id,lat,lon,pois);
        V++;
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

    #if defined(PHASE2)
    int k = min(16, V);
    vector<int> landmark_nodes;
    for (int i = 0; i < k; ++i) {
        landmark_nodes.push_back(i * (V / k));
    }
    g.precomputeLandmarks(landmark_nodes);
    #endif

    ifstream fin2(argv[2], ios::in | ios::binary);
    string buffer2((istreambuf_iterator<char>(fin2)), istreambuf_iterator<char>());
    json q = json::parse(buffer2, nullptr, false);

    json out;
    out["meta"]=q["meta"];
    out["results"]=json::array();
    
    for(auto &query: q["events"]){
        auto start_time=chrono::high_resolution_clock::now();
        json result=processQuery(query,g);
        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        out["results"].push_back(result);
    }

    ofstream fout(argv[3], ios::out | ios::binary);
    fout << out.dump(4);

    return 0;
    
}