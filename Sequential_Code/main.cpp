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
#include <fstream>
#include <chrono>
using namespace std;
using namespace chrono;

void readGraph(const string& filename, vector<vector<pair<int, int>>>& graph) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error: Unable to open the file." << endl;
        return;
    }
    int num_nodes, num_edges;
    file >> num_nodes >> num_edges;  
    graph.resize(num_nodes + 1);  
    int start, end, weight;
    while (file >> start >> end >> weight) {
        graph[start].push_back(make_pair(end, weight));  
    }
    file.close();  
}


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





int main() {
    auto start = high_resolution_clock::now();
    int numVertices = 6;
     vector<vector<pair<int, int>>> graph(numVertices);

     readGraph("small_dataset.txt", graph);
     
     
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

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop-start);
    cout<<"Time taken in squential implementation is: "<<duration.count()<<endl;
    
    return 0;
}

// Large dataset: 114ms
// Medium dataset: 88ms
// Small dataset: 0ms



// Function to run Dijkstra's algorithm and verify results


