#ifndef SOSP_UPDATE_H
#define SOSP_UPDATE_H

#include <vector>
#include <list>
#include <utility>
using namespace std;


class SOSP_Update {
public:
    // Constructor
    SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices);

    // Function to update the graph with new insertions
    void update(const vector<pair<int, int>>& insertions);

    // Function to get the updated distances
    const vector<int>& getDistances() const;

    // Function to get the parent array
    const vector<int>& getParentArray() const;

private:
    int numVertices;
    vector<list<pair<int, int>>> adjacencyList;
    vector<int> distances;
    vector<int> parent;
    vector<int> marked;

    void preprocess(const vector<pair<int, int>>& insertions);
    void processChangedEdges();
    void propagateUpdate(vector<int>& Aff);
};

#endif // SOSP_UPDATE_H