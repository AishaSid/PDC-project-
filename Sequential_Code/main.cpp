#include <iostream>
#include <vector>
#include "MospUpdate.h"
#include "SospUpdate.h"
#include "Mosp_Update.h"
#include "Sosp_Update.h"
#include <queue>
#include <algorithm>
#include <limits>
#include <utility>
#include "metis.h"
using namespace std;

void printGraph(const vector<vector<pair<int, int>>>& graph) {
    for (size_t u = 0; u < graph.size(); ++u) {
        cout << "Vertex " << u + 1 << ": ";
        for (const auto& [v, w] : graph[u]) {
            cout << "(" << v+1 << ", " << w << ") ";
        }
        cout << endl;
    }
}

template <typename GraphType>
void printGraphList(const GraphType& graph) {
    for (size_t u = 0; u < graph.size(); ++u) {
        cout << "V" << u + 1 << " -> ";
        for (const auto& edge : graph[u]) {
            cout << "V" << edge.first + 1 << " (edge = " << edge.second << ") ";
        }
        cout << endl;
    }
}

void runDijkstra(const vector<vector<pair<int, int>>>& graph, int source) {
    int n = graph.size();
    vector<int> dist(n, numeric_limits<int>::max());
    vector<int> parent(n, -1);
    
    dist[source] = 0;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});
    
    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();
        
        if (d > dist[u]) continue;
        
        for (const auto& [v, w] : graph[u]) {
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    
    cout << "Shortest paths from source " << source + 1 << ":" << endl;
    for (int i = 0; i < n; ++i) {
        cout << "To vertex " << i + 1 << ": ";
        if (dist[i] == numeric_limits<int>::max()) {
            cout << "INF" << endl;
        } else {
            cout << dist[i] << " (path: ";
            vector<int> path;
            for (int at = i; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());
            
            for (size_t j = 0; j < path.size(); ++j) {
                cout << path[j] + 1;
                if (j < path.size() - 1) cout << " -> ";
            }
            cout << ")" << endl;
        }
    }
}


void testLargeGraph() {
    int numVertices = 100;
    vector<vector<pair<int, int>>> graph(numVertices);
    srand(42); 
    for (int i = 1; i < numVertices; ++i) {
        int parent = rand() % i;
        int weight = 1 + rand() % 10;
        graph[parent].push_back({i, weight});
        graph[i].push_back({parent, weight});
    }
    int extraEdges = numVertices * 2;
    for (int i = 0; i < extraEdges; ++i) {
        int u = rand() % numVertices;
        int v = rand() % numVertices;
        if (u != v) {
            int weight = 1 + rand() % 10;
            graph[u].push_back({v, weight});
        }
    }
    cout << "Created large graph with " << numVertices << " vertices" << endl;
    vector<int> sources = {0, 1};
    vector<tuple<int, int, int>> insertions;
    for (int i = 0; i < 10; ++i) {
        int u = rand() % numVertices;
        int v = rand() % numVertices;
        int w = 1 + rand() % 5;
        if (u != v) {
            insertions.push_back({u, v, w});
        }
    }
    cout << "Generated " << insertions.size() << " insertions" << endl;
    cout << "\nBefore insertions:" << endl;
    runDijkstra(graph, sources[0]);
    for (const auto& [u, v, w] : insertions) {
        graph[u].push_back({v, w});
    }
    cout << "\nAfter insertions:" << endl;
    runDijkstra(graph, sources[0]);
    
    // Now use your MOSP algorithm here
    // MOSP_Update mosp(graph, sources);
    // mosp.update(insertions);
    
    // Compare results
    // const vector<int>& mosp_dist = mosp.getDistances();
    // const vector<int>& mosp_parent = mosp.getParentArray();
    
    // Verify MOSP results by comparing with Dijkstra
    // (You would need to add verification code here)
    MOSP_Update mosp(graph, sources);
    cout<<"graph made"<<endl;
    mosp.update(insertions);
    cout<<"insertions done"<<endl;
    for (size_t i = 0; i < mosp.ssspTrees.size(); ++i) {
        cout << "SOSP Tree for Source " << i + 1 << ":\n";
        printGraphList(mosp.ssspTrees[i].getGraph());
        cout << endl;
    }
    

    cout<<"Combined Graph: "<<endl; 
    printGraphList(mosp.combinedGraph);
}


int main() {
   
    int numVertices = 6;
     vector<vector<pair<int, int>>> graph(numVertices);
 
     
     graph[0].push_back({2, 8});     // V1 -> V3
     graph[2].push_back({1, 2});     // V3 -> V2
     graph[1].push_back({3, 3});     // V2 -> V4
     graph[1].push_back({4, 9});     // V2 -> V5
     graph[3].push_back({5, 2});     // V4 -> V6
     graph[4].push_back({3, 2});     // V5 -> V4
     graph[4].push_back({5, 6});     // V5 -> V6
 
     // Initialize the MOSP_Update object with multiple sources
     vector<int> sources = {0, 1};
     MOSP_Update mosp(graph, sources);
    cout<<"graph made"<<endl;
     // Insert new edges
     vector<tuple<int, int, int>> insertions = {
        {0, 1, 7},  // V1 -> V2 with weight 7
        {0, 4, 4},   // V1 -> V5 with weight 4
        {2, 4, 1}   // V3 -> V5 with weight 4
     };
 
     // Update the graph with new insertions
     mosp.update(insertions);
     cout<<"insertions done"<<endl;

     // Get the updated distances and parent array
     const vector<int>& distances = mosp.getDistances();
     const vector<int>& parent = mosp.getParentArray();
 

     for (size_t i = 0; i < mosp.ssspTrees.size(); ++i) {
        cout << "SOSP Tree for Source " << i + 1 << ":\n";
        printGraphList(mosp.ssspTrees[i].getGraph());
        cout << endl;
    }
    

    cout<<"Combined Graph: "<<endl; 
    printGraphList(mosp.combinedGraph);

    cout<<"Testing for large graph: "<<endl;
    testLargeGraph();
    return 0;
}






// Function to run Dijkstra's algorithm and verify results

