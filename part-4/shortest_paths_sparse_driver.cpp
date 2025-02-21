#include "shortest_paths_sparse.hpp"
#include "graph.hpp"


int main(int argc, const char *argv[])
{
    //auto graph = sparse_test_graph;
    SparseGraph graph = SparseGraph{
        {{4,1}, {8,7},},
        {{11,7},},
        {{4,5}, {2,8},},
        {{9,4}, {14,5},},
        {{10,5},},
        {{2,6},},
        {{3,3}, {1,7}, {6,8},},
        {{7,8},},
        {}};
    print_graph(graph);

    {
        int source = 2;
        bool has_negative_cycle;
        std::cout << "Bellman-Ford from source " << source
                  << std::endl;
        auto DP = bellman_ford(graph, source, has_negative_cycle);
        for (const auto &dist : DP) {
            std::cout << dist << " ";
        }
        std::cout << std::endl;
    }

    // {
    //     int source = 2;
    //     std::cout << "Dijkstra from source " << source << std::endl;
    //     auto DP = dijkstra(graph, source);
    //     for (const auto &dist : DP) {
    //         std::cout << dist << " ";
    //     }
    //     std::cout << std::endl;
    // }

    return 0;
}