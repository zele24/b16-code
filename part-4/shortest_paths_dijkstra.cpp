#include "shortest_paths_dijkstra.hpp"
#include "shortest_paths_relax.hpp"
#include <priority_queue.hpp>

#include <cmath>

std::vector<hop_t> dijkstra(const Graph &graph, const int source)
{
    const int V = static_cast<int>(graph.size());
    assert(0 <= source && source < V);

    auto DP = std::vector<hop_t>(V, {inf, -1});
    DP[source].weight = 0;

    std::vector<bool> is_open(V, true);

    while (true) {
        float D_star = inf;
        int v_star = -1;
        for (int v = 0; v < V; ++v) {
            if (is_open[v] && DP[v].weight < D_star) {
                D_star = DP[v].weight;
                v_star = v;
            }
        }

        if (v_star < 0) {
            break; // all closed, stop
        }

        is_open[v_star] = false;

        for (int v = 0; v < V; ++v) {
            if (is_open[v] && std::isfinite(graph[v_star][v])) {
                relax(graph, DP, v_star, v);
            }
        }
    }

    return DP;
}
/*
Initialization:

The function dijkstra takes a graph and a source vertex as inputs.
V is set to the number of vertices in the graph.
An assertion ensures that the source vertex is within the valid range.
DP is a vector of hop_t structures, initialized with infinite weight (inf) and a predecessor of -1. The weight for the source vertex is set to 0.
is_open is a boolean vector indicating whether a vertex is still open (i.e., not yet processed).
Main Loop:

The algorithm enters a loop that continues until all vertices are processed.
D_star and v_star are used to find the open vertex with the smallest tentative distance.
A nested loop iterates over all vertices to find the vertex v_star with the smallest weight in DP that is still open.
Processing the Closest Vertex:

If no open vertex is found (v_star < 0), the loop breaks, indicating that all vertices have been processed.
The closest vertex v_star is marked as closed by setting is_open[v_star] to false.
Relaxation:

Another loop iterates over all vertices to relax the edges from v_star to its neighbors.
If a neighbor vertex v is open and there is a finite edge from v_star to v, the relax function is called to update the tentative distances.
Return:

The function returns the DP vector, which contains the shortest path information from the source vertex to all other vertices.
Example
If the graph is represented as an adjacency matrix and the source vertex is 0, the function will compute the shortest paths from vertex 0 to all other vertices using Dijkstra's algorithm.

Key Concepts
Dijkstra's Algorithm: An algorithm for finding the shortest paths between nodes in a graph.
Relaxation: The process of updating the shortest path estimate for a vertex.
Priority Queue: Typically used in Dijkstra's algorithm to efficiently find the vertex with the smallest tentative distance, though this implementation uses a simple linear search.
Potential Improvements
Using a priority queue (e.g., a min-heap) instead of a linear search to find v_star can improve the algorithm's efficiency.
Code Example
This example includes the relax function and a simple graph representation using an adjacency matrix.
*/