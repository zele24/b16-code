#ifndef __shortest_paths_decode__
#define __shortest_paths_decode__

#include "graph.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
using namespace std;

inline std::vector<int> decode(const std::vector<hop_t> &DP, int v)
{
    vector<int> vec = {};
    if (DP[v].weight == inf) {
        return vec;
    }

    // finds the length and previous node of the path
    vec.push_back(v);  //last point on path
    int prev_point = DP[v].vertex;
    while (!(prev_point == -1)) {
        vec.push_back(prev_point);
        prev_point = DP[prev_point].vertex;
    }

    std::reverse(vec.begin(), vec.end());
    return vec;
}


std::vector<std::vector<hop_t>> floyd_warshall(const Graph &graph)
{
    const auto V = static_cast<int>(graph.size());

    auto DP =
        std::vector<std::vector<hop_t>>(V, std::vector<hop_t>(V, {inf, -1}));

    for (int u = 0; u < V; ++u) {
        for (int v = 0; v < V; ++v) {
            if (u == v) {
                DP[u][v].weight = 0;
                DP[u][v].vertex = -1;
            } else if (std::isfinite(graph[u][v])) {
                DP[u][v].weight = graph[u][v];
                DP[u][v].vertex = u;
            }
        }
    }

    for (int r = 0; r < V; ++r) {
        for (int u = 0; u < V; ++u) {
            for (int v = 0; v < V; ++v) {
                auto duv = DP[u][v].weight;
                auto dur = DP[u][r].weight;
                auto drv = DP[r][v].weight;
                if (dur + drv < duv) {
                    DP[u][v].weight = dur + drv;
                    DP[u][v].vertex = DP[r][v].vertex;
                }
            }
        }
    }

    return DP;
}




#endif // __shortest_paths_decode__

int main(int argc, const char *argv[])
{
    Graph graph =
    Graph{{inf , 4  , inf, inf , inf, inf , inf, 8  , inf}  ,
          {inf , inf, inf, inf , inf, inf , inf, 11 , inf}  ,
          {inf , inf, inf, inf , inf, 4   , inf, inf, 2}    ,
          {inf , inf, inf, inf , 9  , 14  , inf, inf, inf}  ,
          {inf , inf, inf, inf , inf, 10  , inf, inf, inf}  ,
          {inf , inf, inf, inf , inf, inf , 2  , inf, inf}  ,
          {inf , inf, inf, 3   , inf, inf , inf, 1  , 6}    ,
          {inf , inf, inf, inf , inf, inf , inf, inf, 7}    ,
          {inf , inf, inf, inf , inf, inf , inf, inf, inf}};
    

    std::cout << "Floyd-Warshall ASPS" << std::endl;
    auto DP = floyd_warshall(graph);
    for (const auto &row : DP) {
        for (const auto& elem : row) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    for (int u = 0; u < (signed)graph.size(); ++u) {
        for (int v = 0; v < (signed)graph.size(); ++v) {
            auto path = decode(DP[u], v);
            if (path.size()) {
                std::cout << "Shortest path " << u << " ~~> " << v
                          << " (weight " << DP[u][v].weight << "): ";
                for (const auto& elem : path) {
                    std::cout << elem << " ";
                }
                std::cout << std::endl;
            }
        }
    };

    return 0;
}
