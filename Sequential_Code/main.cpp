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

void printGraphList(const vector<list<pair<int, int>>>& graph) {
    for (size_t u = 0; u < graph.size(); ++u) {
        cout << "V" << u + 1 << " -> ";
        for (const auto& [v, w] : graph[u]) {
            cout << "V" << v + 1 << " (edge = " << w << "), ";
        }
        cout << endl;
    }
}


int main() {
    int numVertices = 6;
    vector<vector<pair<int, int>>> graph(numVertices);

    // Define edges as per your instructions
    graph[0].push_back({2, 8});     // V1 -> V3
    graph[2].push_back({1, 2});     // V3 -> V2
    graph[1].push_back({3, 3});     // V2 -> V4
    graph[1].push_back({4, 9});     // V2 -> V5
    graph[3].push_back({5, 2});     // V4 -> V6
    graph[4].push_back({3, 2});     // V5 -> V4
    graph[4].push_back({5, 6});     // V5 -> V6

    cout << "Graph before SOSP update:\n";
    printGraph(graph);

    // Initialize SOSP_Update
    SOSP_Update sosp(graph, numVertices);

    // Edges to insert (you can modify this if needed)
    vector<tuple<int, int, int>> insertions = {
        {0, 1, 7},  // V1 -> V2 with weight 7
        {0, 4, 4},   // V1 -> V5 with weight 4
        {2, 4, 1}   // V3 -> V5 with weight 4
    };

    sosp.update(insertions);

    cout << "\nGraph after SOSP update:\n";
    printGraphList(sosp.getGraph());

    cout << "\nUpdated Distances: ";
    for (int d : sosp.getDistances()) {
        cout << d << " ";
    }
    cout << "\nParent Array: ";
    for (int p : sosp.getParentArray()) {
        cout << p + 1 << " ";  
    }
    cout << endl;

    return 0;
}
