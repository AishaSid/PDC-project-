#ifndef MOSP_UPDATE_H
#define MOSP_UPDATE_H

#include <vector>
#include <list>
#include <utility>
#include <tuple>
#include "SospUpdate.h"
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

    vector<SOSP_Update> ssspTrees; 


    vector<vector<pair<int, int>>> combinedGraph; 
    int numVertices;
    private:   
    vector<int> distances;
    vector<int> parent;

    void findUpdatedSOSPTree(const vector<tuple<int, int, int>>& insertions, int treeIndex);
    void createCombinedGraph();
    void findSOSPInCombinedGraph();
};

#endif // MOSP_UPDATE_H