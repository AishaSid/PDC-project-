#include "MospUpdate.h"
#include "SospUpdate.h"
#include <queue>
#include <algorithm>
#include <limits>
#include <iostream>
#include <vector>
#include <utility> 
#include <tuple>
using namespace std;

MOSP_Update::MOSP_Update(const vector<vector<pair<int, int>>>& graph, const vector<int>& sources)
    : numVertices(graph.size()), ssspTrees(sources.size(), graph), distances(numVertices, numeric_limits<int>::max()), parent(numVertices, -1) {
}

void MOSP_Update::update(const vector<tuple<int, int, int>>& insertions) 
{
    for (size_t i = 0; i < insertions.size(); ++i) 
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
    cout<<"Entered findUpdatedSOSPTree function for tree: "<<treeIndex<<endl;
    SOSP_Update sospUpdate(ssspTrees[treeIndex], numVertices);
    sospUpdate.update(insertions);
    distances = sospUpdate.getDistances();
    parent = sospUpdate.getParentArray();
}

void MOSP_Update::createCombinedGraph()
 {
    combinedGraph.assign(numVertices, vector<pair<int, int>>());

    for (const auto& tree : ssspTrees) 
    {
        for (int u = 0; u < numVertices; ++u) 
        {
            for (const auto& edge : tree[u]) 
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
            for (const auto& tree : ssspTrees) {
                if (find(tree[u].begin(), tree[u].end(), edge) != tree[u].end()) {
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

        for (const auto& edge : ssspTrees[0][u])
         { // Use the first SSSP tree for demonstration
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