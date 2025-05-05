#include "MospUpdate.h"
#include "SospUpdate.h"
#include <queue>
#include <mpi.h>
#include <algorithm>
#include <limits>
#include <iostream>
#include <vector>
#include <utility> 
#include <tuple>
using namespace std;

// Declare combinedGraph as a member variable

MOSP_Update::MOSP_Update(const vector<vector<pair<int, int>>>& graph, const vector<int>& sources)
    : numVertices(graph.size()), 
      distances(numVertices, numeric_limits<int>::max()), 
      parent(numVertices, -1) {
    for (size_t i = 0; i < sources.size(); ++i) {
        ssspTrees.emplace_back(graph, numVertices);
    }
}

void MOSP_Update::update(const vector<tuple<int, int, int>>& insertions) 
{
    for (size_t i = 0; i < ssspTrees.size(); ++i) // Loop over the size of sources (or ssspTrees)
    {
        findUpdatedSOSPTree(insertions, i);
    }
    createCombinedGraph();
    findSOSPInCombinedGraph();
}

const vector<int>& MOSP_Update::getDistances() const
{
    return distances;
}

const vector<int>& MOSP_Update::getParentArray() const 
{
    return parent;
}

void MOSP_Update::findUpdatedSOSPTree(const vector<tuple<int, int, int>>& insertions, int treeIndex) 
{
    cout << "Entered findUpdatedSOSPTree function for tree: " << treeIndex << endl;
    ssspTrees[treeIndex].update(insertions); // Directly update the corresponding SOSP_Update instance
    ssspTrees[treeIndex].removeAffectedEdges(); // Remove affected edges from the tree
    cout << "Updated SOSP tree for source: " << treeIndex << endl;
    cout << "Parent Array after update: ";
    for (int i = 0; i < numVertices; ++i) {
        cout << ssspTrees[treeIndex].getParentArray()[i] << " ";
    } cout << endl;
}

void MOSP_Update::createCombinedGraph()
{
    combinedGraph.assign(numVertices, vector<pair<int, int>>());

    for (const auto& sospTree : ssspTrees) 
    {
        const auto& graph = sospTree.getGraph(); // Get the adjacency list from SOSP_Update
        for (int u = 0; u < numVertices; ++u) 
        {
            for (const auto& edge : graph[u]) 
            {
                combinedGraph[u].push_back(edge);
            }
        }
    }

    // Assign weights based on the number of trees each edge appears in
    for (int u = 0; u < numVertices; ++u) 
    {
        for (auto& edge : combinedGraph[u]) {
            int count = 0;
            for (const auto& sospTree : ssspTrees) {
                const auto& graph = sospTree.getGraph();
                if (find(graph[u].begin(), graph[u].end(), edge) != graph[u].end()) {
                    ++count;
                }
            }
            edge.second = (ssspTrees.size() - count + 1);
        }
    }

    // Update the combined graph with the new weights
    for (int u = 0; u < numVertices; ++u) 
    {
        for (auto& edge : combinedGraph[u]) 
        {
            edge.second = 1; // Assign actual edge weights
        }
    }
}

void MOSP_Update::findSOSPInCombinedGraph() 
{
    // Implement Dijkstra's algorithm or another shortest path algorithm to find the SOSP in the combined graph
    // This is a placeholder for the actual implementation
    fill(distances.begin(), distances.end(), numeric_limits<int>::max());
    distances[0] = 0; // Assuming source is vertex 0
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, 0});

    while (!pq.empty()) 
    {
        int dist = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (dist > distances[u]) continue;

        for (const auto& edge : combinedGraph[u]) 
        {
            int v = edge.first;
            int weight = edge.second;
            if (distances[v] > distances[u] + weight) 
            {
                distances[v] = distances[u] + weight;
                parent[v] = u;
                pq.push({distances[v], v});
            }
        }
    }
}