#ifndef SOSP_UPDATE_H
#define SOSP_UPDATE_H

#include <vector>
#include <list>
#include <utility>
#include <tuple>
using namespace std;


class SOSP_Update {
public:
    // Constructor
    SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices);
    
    void update(const vector<tuple<int, int, int>>& insertions);
    
    const vector<int>& getDistances() const;
   
    const vector<int>& getParentArray() const;

private:
    int numVertices;
    vector<list<pair<int, int>>> adjacencyList;
    vector<list<pair<int, int>>> reverseAdjList;

    vector<int> distances;
    vector<int> parent;
    vector<int> marked;

    vector<list<pair<int, int>>> preprocess(const vector<tuple<int, int, int>>& insertions);
    
    void processChangedEdges(const vector<list<pair<int, int>>>& I);
    void propagateUpdate(vector<int>& Aff);
};

#endif // SOSP_UPDATE_H