#include "SospUpdate.h"
#include <queue>
#include <algorithm>
#include <limits>

SOSP_Update::SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices)
    : numVertices(numVertices), adjacencyList(numVertices), distances(numVertices, numeric_limits<int>::max()), parent(numVertices, -1), marked(numVertices, 0) {
    // Initialize the adjacency list with the given graph
    for (int u = 0; u < numVertices; ++u) {
        for (const auto& edge : graph[u]) {
            int v = edge.first;
            int weight = edge.second;
            adjacencyList[u].push_back({v, weight});
        }
    }
}

void SOSP_Update::update(const vector<pair<int, int>>& insertions) {
    preprocess(insertions);
    processChangedEdges();
}

const vector<int>& SOSP_Update::getDistances() const {
    return distances;
}

const vector<int>& SOSP_Update::getParentArray() const {
    return parent;
}

void SOSP_Update::preprocess(const vector<pair<int, int>>& insertions) {
    // Initialize an array I of size |V| where each element is an empty list
    vector<list<pair<int, int>>> I(numVertices);

    // Add each edge (u, v) to I[v]
    for (const auto& edge : insertions) {
        int u = edge.first;
        int v = edge.second;
        I[v].push_back({u, v});
    }
}

void SOSP_Update::processChangedEdges() {
    // Initialize an empty vector Aff
    vector<int> Aff;

    // Initialize an array marked containing zeroes of size |V|
    marked.assign(numVertices, 0);

    // Process each vertex in parallel
    for (int v = 0; v < numVertices; ++v) {
        for (const auto& edge : adjacencyList[v]) {
            int u = edge.first;
            int weight = edge.second;

            if (distances[v] > distances[u] + weight) {
                Aff.push_back(v);
                distances[v] = distances[u] + weight;
                parent[v] = u;
                marked[v] = 1;
            }
        }
    }

    // Propagate the update
    propagateUpdate(Aff);
}

void SOSP_Update::propagateUpdate(vector<int>& Aff) {
    while (!Aff.empty()) {
        vector<int> N;
        vector<int> AffPrime;

        // Add the neighbors of each vertex in Aff to N
        for (int v : Aff) {
            for (const auto& edge : adjacencyList[v]) {
                N.push_back(edge.first);
            }
        }

        // Process each vertex in N in parallel
        for (int v : N) {
            for (const auto& edge : adjacencyList[v]) {
                int u = edge.first;
                int weight = edge.second;

                if (marked[u] != 1) {
                    continue;
                }

                if (distances[v] > distances[u] + weight) {
                    AffPrime.push_back(v);
                    distances[v] = distances[u] + weight;
                    parent[v] = u;
                    marked[v] = 1;
                }
            }
        }

        Aff = AffPrime;
        AffPrime.clear();
    }
}