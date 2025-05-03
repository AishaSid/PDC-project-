#include "SospUpdate.h"
#include <queue>
#include <algorithm>
#include <limits>
#include <list>
#include <tuple>
#include <vector>
#include <iostream>
using namespace std;

SOSP_Update::SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices)
    : numVertices(numVertices),
      adjacencyList(numVertices),
      distances(numVertices, numeric_limits<int>::max()),
      parent(numVertices, -1),
      marked(numVertices, 0),
      reverseAdjList(numVertices)
{
    // Convert vector<vector<>> to vector<list<>>
    for (int u = 0; u < numVertices; ++u) {
        for (const auto& [v, w] : graph[u]) {
            adjacencyList[u].emplace_back(v, w);
            reverseAdjList[v].emplace_back(u, w);  // Build reverse graph
        }
    }

    // Set the source distance to 0
    if (!distances.empty()) {
        distances[0] = 0;  // Assuming the source vertex is 0
    }
}

const std::vector<int>& SOSP_Update::getDistances() const
{
    return distances;
}

const std::vector<int>& SOSP_Update::getParentArray() const 
{
    return parent;
}

vector<list<pair<int, int>>> SOSP_Update::preprocess(const vector<tuple<int, int, int>>& insertions) 
{
    vector<list<pair<int, int>>> I(numVertices);

    
    for (const auto& [u, v, w] : insertions) 
    {
        adjacencyList[u].push_back({v, w});
       // I[v].push_back({u, w});
    }

    // Add insertions to I
    for (int u = 0; u < numVertices; ++u) 
    {
        for (const auto& [v, w] : adjacencyList[u])
        {
            I[v].push_back({u, w});  // Correct: edge from u to v
        }
    }
    
    // Output the array I for debugging
    for (int i = 0; i < I.size(); ++i) 
    {
        cout << "I[" << i << "]: ";
        for (const auto& [u, w] : I[i]) 
        {
            cout << "(" << u << ", " << w << ") ";
        }
        cout << endl;
    }

    return I;
}

void SOSP_Update::update(const vector<tuple<int, int, int>>& insertions)
{
    vector<list<pair<int, int>>> I = preprocess(insertions);
    processChangedEdges(I);
}

void SOSP_Update::processChangedEdges(const vector<list<pair<int, int>>>& I)
{
    vector<int> Aff;
    marked.assign(numVertices, 0);

    for (int v = 0; v < numVertices; ++v)
     {
        for (const auto& [u, _] : I[v])
         {
            auto it = find_if(adjacencyList[u].begin(), adjacencyList[u].end(),
                              [v](const pair<int, int>& p) { return p.first == v; });

            if (it != adjacencyList[u].end())
            {
                int weight = it->second;
                if (distances[v] > distances[u] + weight)
                 {
                    distances[v] = distances[u] + weight;
                    parent[v] = u;
                    marked[v] = 1;
                    Aff.push_back(v);
                }
            }
        }
    }

    propagateUpdate(Aff);
}

void SOSP_Update::propagateUpdate(vector<int>& Aff) 
{
    while (!Aff.empty()) 
    {
        vector<int> N;
        vector<int> AffPrime;

        for (int v : Aff) 
        {
            for (const auto& [nbr, _] : adjacencyList[v]) 
            {
                N.push_back(nbr);
            }
        }

        for (int v : N)
         {
            for (const auto& [u, w] : reverseAdjList[v]) 
            {
                if (!marked[u]) 
                    continue;

                if (distances[v] > distances[u] + w) {
                    distances[v] = distances[u] + w;
                    parent[v] = u;
                    marked[v] = 1;
                    AffPrime.push_back(v);
                }
            }
        }

        Aff = move(AffPrime);
    }
}
