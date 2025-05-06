#include "SospUpdate.h"
#include <queue>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <list>
#include <tuple>
#include <vector>
#include <iostream>
using namespace std;

void SOSP_Update::updateParentAndDistance()
{
    for (int u = 0; u < numVertices; ++u) {
        for (const auto& [v, w] : adjacencyList[u]) {
            if (distances[u] != numeric_limits<int>::max() && distances[v] > distances[u] + w) {
                distances[v] = distances[u] + w;
                parent[v] = u;
            }
        }
    }
}


SOSP_Update::SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices)
    : numVertices(numVertices),
      adjacencyList(numVertices),
      distances(numVertices, numeric_limits<int>::max()),
      parent(numVertices, -1),
      marked(numVertices, 0),
      reverseAdjList(numVertices)
{
    for (int u = 0; u < numVertices; ++u) {
        for (const auto& [v, w] : graph[u]) {
            adjacencyList[u].emplace_back(v, w);
            reverseAdjList[v].emplace_back(u, w);
        }
    }

    if (!distances.empty()) {
        distances[0] = 0;  // Source node is assumed to be 0
    }
}

const vector<int>& SOSP_Update::getDistances() const { return distances; }
const vector<int>& SOSP_Update::getParentArray() const { return parent; }

void SOSP_Update::update(const vector<tuple<int, int, int>>& insertions)
{
    marked.assign(numVertices, 0);
    vector<int> Aff;

    
    for (const auto& [u, v, w] : insertions)
    {
        // Add to graph and reverse graph
        adjacencyList[u].emplace_back(v, w);
        reverseAdjList[v].emplace_back(u, w);

        if (distances[u] != numeric_limits<int>::max() && distances[v] > distances[u] + w)
        {
            distances[v] = distances[u] + w;
            parent[v] = u;
            if (!marked[v]) {
                marked[v] = 1;
                Aff.push_back(v);
            }
        }
    }

    
    // Step 1: Print the graph
    cout << "Step 1 :" << endl;
    for (int u = 0; u < numVertices; ++u) 
    {
        cout << "Node " << u << ":";
        for (const auto& [v, w] : adjacencyList[u]) 
        {
            cout << " -> (" << v << ", " << w << ")";
        }
        cout << endl;
    }
    // print the parent array
    cout << "Parent Array before update: ";       
    for (int i = 0; i < numVertices; ++i) {
        cout << parent[i] << " ";
    }cout << endl;

   // updateParentAndDistance();

    // cout << "Parent Array after update: ";       
    // for (int i = 0; i < numVertices; ++i) {
    //     cout << parent[i] << " ";
    // }cout << endl;

    // Print Affected edges
    cout << "Affected Edges: ";
    for (int i = 0; i < Aff.size(); ++i) {
        cout << Aff[i] << " ";
    }cout <<endl;



    propagateUpdate(Aff);
}

void SOSP_Update::propagateUpdate(vector<int>& Aff)
{
    unordered_set<int> N;
        int m = 0; 
    while (!Aff.empty())
    {
        m++;
        vector<int> AffPrime;
        N.clear();

        for (int v : Aff)
        {
            for (const auto& [nbr, _] : adjacencyList[v])
                N.insert(nbr);
        }

        for (int v : N)
        {
            for (const auto& [u, w] : reverseAdjList[v])
            {
                if (!marked[u]) continue;

                if (distances[v] > distances[u] + w)
                {
                    distances[v] = distances[u] + w;
                    parent[v] = u;
                    if (!marked[v]) {
                        marked[v] = 1;
                        AffPrime.push_back(v);
                    }
                }
            }
        }

        Aff = move(AffPrime);
               // Print Affected edges
    cout << "Affected Edges: at iteration "<< m << ": ";
    for (int i = 0; i < Aff.size(); ++i) {
        cout << Aff[i] << " ";
    } cout <<endl; 
    }

    updateParentAndDistance();
}

void SOSP_Update::removeAffectedEdges()
{
    for (int u = 0; u < numVertices; ++u) {
        auto& edges = adjacencyList[u];
        for (auto it = edges.begin(); it != edges.end(); ) {
            int v = it->first;
            int w = it->second;

            if (distances[v] != distances[u] + w || parent[v] != u) {
                it = edges.erase(it);
            } else {
                ++it;
            }
        }
    }
}
