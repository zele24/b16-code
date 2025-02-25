#include "shortest_paths_sparse.hpp"
#include "graph.hpp"

#include <cmath>
using namespace std;

std::vector<hop_t> bellman_ford(const SparseGraph &graph, const int source,
                                bool &has_negative_cycle)
{
    const int V = static_cast<int>(graph.size());
    
    auto DP = std::vector<hop_t>(V, {inf, -1});


    DP[source].weight = 0;

    /*
    /////////////////// Initial testing /////////////////////
    // // Nodes connected to node 2 will have weight = weight 
    // // hop_t have .vertex and .weight (both integers)
    // // Iterate over very length and see if node connect to other (unconnected) nodes in that length

    // //directly connected:
    // for (int i = 0; i<graph[source].size(); i++) {
    //     if (graph[source][i].weight < DP[graph[source][i].vertex].weight) {
    //         DP[graph[source][i].vertex].weight = graph[source][i].weight;
    //     }
    // }

    // // 2 away
    // for (int i = 0; i<graph[source].size(); i++) {
    //     for (int j = 0; j<graph[graph[source][i].vertex].size(); j++) {
    //         if (graph[source][i].weight + graph[graph[source][i].vertex][j].weight < DP[graph[graph[source][i].vertex][j].vertex].weight) {
    //             DP[graph[graph[source][i].vertex][j].vertex].weight = graph[source][i].weight + graph[graph[source][i].vertex][j].weight;
    //         }
    //     }
    // }

    // // 3 away
    // for (int i = 0; i<graph[source].size(); i++) {
    //     //node of conern
    //     hop_t node = graph[source][i];
    //     for (int j = 0; j<graph[node.vertex].size(); j++) {
    //         node = graph[node.vertex][j];
    //         for (int k = 0; k<graph[node.vertex].size(); k++) {
    //             if (graph[source][i].weight + graph[graph[source][i].vertex][j].weight + graph[graph[graph[source][i].vertex][j].vertex][k].weight < DP[graph[graph[graph[source][i].vertex][j].vertex][k].vertex].weight) {
    //                 DP[graph[graph[graph[source][i].vertex][j].vertex][k].vertex].weight = graph[source][i].weight + graph[graph[source][i].vertex][j].weight + graph[graph[graph[source][i].vertex][j].vertex][k].weight;
    //             }
    //         }
    //     }    
    // }


    
    // paths will store the weights of each path, in vectors of number of nodes along each path
    
    
    // //Process to get all paths of length 2
    // // looping over each element in the first vector<hop_t> of paths (going along each node with path length 1)
    // for (int j = 0; j < paths[0].size(); j++){
    // hop_t temp_node;
    // vector<hop_t> graph_node = graph[paths[0][j].vertex];
    // // Taking every connection from that node and creating a path of length 2, stoing it in the next paths vector
    // for (int k = 0; k < graph_node.size(); k++){
    //     temp_node.vertex = graph_node[k].vertex;
    //     temp_node.weight = graph_node[k].weight + paths[0][j].weight;
    //     paths[1].push_back(temp_node);
    // }
    // }
    */
    
    vector<vector<hop_t>> paths(V);
    vector<vector<int>> paths_prev(V); // stores preious nodes

    //Process to get all paths of length 1
    for (int i = 0; i < graph[source].size(); i++){
        hop_t node;
        node.weight = graph[source][i].weight;
        node.vertex = graph[source][i].vertex;
        paths[0].push_back(node);
        paths_prev[0].push_back(source);
    }
    //Process to get all paths of length up to V
    for (int v = 0; v < V-1; v++) {
    // Iterate over each path created previously to get the next nodes possible, and store these with their weights
        for (int j = 0; j < paths[v].size(); j++){
            hop_t temp_node;
            vector<hop_t> graph_node = graph[paths[v][j].vertex];
            // Taking every connection from that node and creating a path 1 longer, storing it in the next paths vector
            for (int k = 0; k < graph_node.size(); k++){
                temp_node.vertex = graph_node[k].vertex; //Just stores the final node in the path for checking later
                temp_node.weight = graph_node[k].weight + paths[v][j].weight; //Summing up the total weight of that new path
                paths_prev[v+1].push_back(paths[v][j].vertex);
                
                paths[v+1].push_back(temp_node);
            }
            }

        }

    // Iterating over all the paths to get the minimum weight for each node
    // This could be done in the earlier loops, but this is more understandable for me

    for (int i = 0; i < V; i++){
        for (int j = 0; j < paths[i].size(); j++){
            if (paths[i][j].weight < DP[paths[i][j].vertex].weight){
                DP[paths[i][j].vertex].weight = paths[i][j].weight;
                DP[paths[i][j].vertex].vertex = paths_prev[i][j]; // Update the value of the previous node
            }
        }
    }

    //

    return DP;
}

struct triplet_t {
    float d;
    int r;
    int v;
};

std::vector<hop_t> dijkstra(const SparseGraph &graph, const int source)
{
  
    const int V = static_cast<int>(graph.size());
    auto DP = std::vector<hop_t>(V, {inf, -1});


    // All nodes are curently open
    std::vector<bool> is_open(V, true);

    // Source is closed with 0 weight
    DP[source].weight = 0;
    is_open[source] = false;

    while (true) {
        float D_star = inf;  //Distance to v_star
        int v_star = -1;  //Open vertex with the smallest total distance
        int prev_node = -1;
        
        //Checks the total distance of all open nodes connected to any closed node, and saves the shortest total path
        for (int v = 0; v < V; ++v) {
            if (!is_open[v]) { 
                for (int j=0; j< graph[v].size(); j++) {
                    if (graph[v][j].weight + DP[v].weight < D_star && is_open[graph[v][j].vertex]) {
                        D_star = graph[v][j].weight + DP[v].weight;  //Weight is the weight of the closed node + weight to the open node
                        v_star = graph[v][j].vertex;
                        prev_node = v;
                    }
                }
            }
        }

        if (v_star < 0) {
            break; // all closed, stop
        }

        DP[v_star].weight = D_star;
        DP[v_star].vertex = prev_node;
        is_open[v_star] = false;

   
    }


    return DP;
}


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

    {
        int source = 2;
        std::cout << "Dijkstra from source " << source << std::endl;
        auto DP = dijkstra(graph, source);
        for (const auto &dist : DP) {
            std::cout << dist << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}