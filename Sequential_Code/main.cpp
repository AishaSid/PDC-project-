#include <iostream>
#include <vector>
#include "SospUpdate.h"
#include <tuple>
using namespace std;

int main() {
    
   
    int numVertices = 5;
    vector<vector<pair<int, int>>> graph(numVertices);

    
    graph[0].push_back({1, 1});
    graph[0].push_back({2, 4});
    graph[1].push_back({2, 2});
    graph[1].push_back({3, 5});
    graph[2].push_back({3, 1});
    graph[3].push_back({4, 3});

    // Initialize the SOSP_Update object
    SOSP_Update sosp(graph, numVertices);

    // Insert new edges
    vector<tuple<int, int, int>> insertions = {{0, 3, 2}, {1, 4, 4}};

    // Update the graph with new insertions
    sosp.update(insertions);

    // Get the updated distances and parent array
    const vector<int>& distances = sosp.getDistances();
    const vector<int>& parent = sosp.getParentArray();

    // Print the results
    cout << "Updated Distances: ";
    for (int dist : distances) {
        cout << dist << " ";
    }
    cout << endl;

    cout << "Parent Array: ";
    for (int p : parent) {
        cout << p << " ";
    }
    cout << endl;

    return 0;
}