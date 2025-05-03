#ifndef MOSP_UPDATE_H
#define MOSP_UPDATE_H

#include <vector>
#include <list>
#include <utility>
#include <tuple>
using namespace std;

class MOSP_Update {
public:
    // Constructor
    MOSP_Update(const vector<vector<pair<int, int>>>& graph, const vector<int>& sources);

    // Function to update the graph with new insertions for multiple SSSP trees
    void update(const vector<tuple<int, int, int>>& insertions);

    // Function to get the updated distances
    const vector<int>& getDistances() const;

    // Function to get the parent array
    const vector<int>& getParentArray() const;

private:
    int numVertices;
    vector<vector<vector<pair<int, int>>>> ssspTrees; // Multiple SSSP trees
    vector<int> distances;
    vector<int> parent;

    void findUpdatedSOSPTree(const vector<tuple<int, int, int>>& insertions, int treeIndex);
    void createCombinedGraph();
    void findSOSPInCombinedGraph();
};

#endif // MOSP_UPDATE_H