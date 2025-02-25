
#include <algorithm>
#include <iostream>
#include <vector>


#include "shortest_paths_decode.hpp"
#include "shortest_paths_fw.hpp"

// int main(int argc, const char *argv[])
// {
//      SparseGraph graph = SparseGraph{
//         {{4,1}, {8,7},},
//         {{11,7},},
//         {{4,5}, {2,8},},
//         {{9,4}, {14,5},},
//         {{10,5},},
//         {{2,6},},
//         {{3,3}, {1,7}, {6,8},},
//         {{7,8},},
//         {}};
    

//     std::cout << "Floyd-Warshall ASPS" << std::endl;
//     auto DP = floyd_warshall(graph);
//     for (const auto &row : DP) {
//         for (const auto& elem : row) {
//             std::cout << elem << " ";
//         }
//         std::cout << std::endl;
//     }
//     std::cout << std::endl;

//     for (int u = 0; u < (signed)graph.size(); ++u) {
//         for (int v = 0; v < (signed)graph.size(); ++v) {
//             auto path = decode(DP[u], v);
//             if (path.size()) {
//                 std::cout << "Shortest path " << u << " ~~> " << v
//                           << " (weight " << DP[u][v].weight << "): ";
//                 for (const auto& elem : path) {
//                     std::cout << elem << " ";
//                 }
//                 std::cout << std::endl;
//             }
//         }
//     };

//     return 0;
// }
