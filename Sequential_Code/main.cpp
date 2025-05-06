#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <utility>
#include <fstream>
#include <queue>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <list>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <limits>
#include <iostream>
#include <vector>
#include <utility> 
#include <tuple>
using namespace std;

// Declare combinedGraph as a member variable

class SOSP_Update {
    public:
        // Constructor
        SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices);
        
        void update(const vector<tuple<int, int, int>>& insertions);
        
        const vector<int>& getDistances() const;
       
        const vector<int>& getParentArray() const;
        const vector<list<pair<int, int>>>& getGraph() const {
            return adjacencyList;
        }
        
        void removeAffectedEdges();
    private:
        int numVertices;
        vector<list<pair<int, int>>> adjacencyList;
        vector<list<pair<int, int>>> reverseAdjList;
    
        vector<int> distances;
        vector<int> parent;
        vector<int> marked;
        void updateParentAndDistance();
    
        vector<list<pair<int, int>>> preprocess(const vector<tuple<int, int, int>>& insertions);
        
        void processChangedEdges(const vector<list<pair<int, int>>>& I);
        void propagateUpdate(vector<int>& Aff);
    };

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
void readGraph(const string& filename, vector<vector<pair<int, int>>>& graph) {
    ifstream file(filename);
    if (!file) {
        cerr << "Error: Unable to open the file." << endl;
        return;
    }
    int num_nodes, num_edges;
    file >> num_nodes >> num_edges;  
    graph.resize(num_nodes + 1);  
    int start, end, weight;
    while (file >> start >> end >> weight) {
        graph[start].push_back(make_pair(end, weight));  
    }
    file.close();  
}


void printGraph(const vector<vector<pair<int, int>>>& graph) {
    for (size_t u = 0; u < graph.size(); ++u) {
        cout << "Vertex " << u + 1 << ": ";
        for (const auto& [v, w] : graph[u]) {
            cout << "(" << v+1 << ", " << w << ") ";
        }
        cout << endl;
    }
}

template <typename GraphType>
void printGraphList(const GraphType& graph) {
    for (size_t u = 0; u < graph.size(); ++u) {
        cout << "V" << u + 1 << " -> ";
        for (const auto& edge : graph[u]) {
            cout << "V" << edge.first + 1 << " (edge = " << edge.second << ") ";
        }
        cout << endl;
    }
}

void runDijkstra(const vector<vector<pair<int, int>>>& graph, int source) {
    int n = graph.size();
    vector<int> dist(n, numeric_limits<int>::max());
    vector<int> parent(n, -1);
    
    dist[source] = 0;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, source});
    
    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();
        
        if (d > dist[u]) continue;
        
        for (const auto& [v, w] : graph[u]) {
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    
    cout << "Shortest paths from source " << source + 1 << ":" << endl;
    for (int i = 0; i < n; ++i) {
        cout << "To vertex " << i + 1 << ": ";
        if (dist[i] == numeric_limits<int>::max()) {
            cout << "INF" << endl;
        } else {
            cout << dist[i] << " (path: ";
            vector<int> path;
            for (int at = i; at != -1; at = parent[at]) {
                path.push_back(at);
            }
            reverse(path.begin(), path.end());
            
            for (size_t j = 0; j < path.size(); ++j) {
                cout << path[j] + 1;
                if (j < path.size() - 1) cout << " -> ";
            }
            cout << ")" << endl;
        }
    }
}


void testLargeGraph() {
    int numVertices = 100;
    vector<vector<pair<int, int>>> graph(numVertices);
    srand(42); 
    for (int i = 1; i < numVertices; ++i) {
        int parent = rand() % i;
        int weight = 1 + rand() % 10;
        graph[parent].push_back({i, weight});
        graph[i].push_back({parent, weight});
    }
    int extraEdges = numVertices * 2;
    for (int i = 0; i < extraEdges; ++i) {
        int u = rand() % numVertices;
        int v = rand() % numVertices;
        if (u != v) {
            int weight = 1 + rand() % 10;
            graph[u].push_back({v, weight});
        }
    }
    cout << "Created large graph with " << numVertices << " vertices" << endl;
    vector<int> sources = {0, 1};
    vector<tuple<int, int, int>> insertions;
    for (int i = 0; i < 10; ++i) {
        int u = rand() % numVertices;
        int v = rand() % numVertices;
        int w = 1 + rand() % 5;
        if (u != v) {
            insertions.push_back({u, v, w});
        }
    }
    cout << "Generated " << insertions.size() << " insertions" << endl;
    cout << "\nBefore insertions:" << endl;
    runDijkstra(graph, sources[0]);
    for (const auto& [u, v, w] : insertions) {
        graph[u].push_back({v, w});
    }
    cout << "\nAfter insertions:" << endl;
    runDijkstra(graph, sources[0]);
    
    // Now use your MOSP algorithm here
    // MOSP_Update mosp(graph, sources);
    // mosp.update(insertions);
    
    // Compare results
    // const vector<int>& mosp_dist = mosp.getDistances();
    // const vector<int>& mosp_parent = mosp.getParentArray();
    
    // Verify MOSP results by comparing with Dijkstra
    // (You would need to add verification code here)
    MOSP_Update mosp(graph, sources);
    cout<<"graph made"<<endl;
    mosp.update(insertions);
    cout<<"insertions done"<<endl;
    for (size_t i = 0; i < mosp.ssspTrees.size(); ++i) {
        cout << "SOSP Tree for Source " << i + 1 << ":\n";
        printGraphList(mosp.ssspTrees[i].getGraph());
        cout << endl;
    }
    

    cout<<"Combined Graph: "<<endl; 
    printGraphList(mosp.combinedGraph);
}


int main() {
   
    int numVertices = 6;
     vector<vector<pair<int, int>>> graph(numVertices);
 
     
     graph[0].push_back({2, 8});     // V1 -> V3
     graph[2].push_back({1, 2});     // V3 -> V2
     graph[1].push_back({3, 3});     // V2 -> V4
     graph[1].push_back({4, 9});     // V2 -> V5
     graph[3].push_back({5, 2});     // V4 -> V6
     graph[4].push_back({3, 2});     // V5 -> V4
     graph[4].push_back({5, 6});     // V5 -> V6
 
     // Initialize the MOSP_Update object with multiple sources
     vector<int> sources = {0, 1};
     MOSP_Update mosp(graph, sources);
    cout<<"graph made"<<endl;
     // Insert new edges
     vector<tuple<int, int, int>> insertions = {
        {0, 1, 7},  // V1 -> V2 with weight 7
        {0, 4, 4},   // V1 -> V5 with weight 4
        {2, 4, 1}   // V3 -> V5 with weight 4
     };
 
     // Update the graph with new insertions
     mosp.update(insertions);
     cout<<"insertions done"<<endl;

     // Get the updated distances and parent array
     const vector<int>& distances = mosp.getDistances();
     const vector<int>& parent = mosp.getParentArray();
 

     for (size_t i = 0; i < mosp.ssspTrees.size(); ++i) {
        cout << "SOSP Tree for Source " << i + 1 << ":\n";
        printGraphList(mosp.ssspTrees[i].getGraph());
        cout << endl;
    }
    

    cout<<"Combined Graph: "<<endl; 
    printGraphList(mosp.combinedGraph);

    cout<<"Testing for large graph: "<<endl;
    testLargeGraph();
    return 0;
}






// Function to run Dijkstra's algorithm and verify results


