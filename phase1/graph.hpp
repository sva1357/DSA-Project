#include<string>
#include<vector>
#include<algorithm>
#include<cmath>

class Graph{

};

class Node{
    int id;
    double lat;
    double lon;
    std::vector<std::string> pois;

    double distance(const Node* other){
        double x=this->lat-other->lat;
        double y=this->lon-other->lon;
        return sqrt(x*x+y*y);
    }

};
class Edge{
    int id;
    int u;
    int v;
    double len;
    double avg_time;
    std::vector<int> spped_profile;
    bool oneway;
    std::string  road_type;




};
