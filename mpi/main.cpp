#include <iostream>
#include <vector>
#include <mpi.h>
#include <queue>
#include <algorithm>
#include <limits>
#include <list>
#include <utility>
#include <fstream>
using namespace std;
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <map>


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


class SOSP_Update {
    public:
        // Constructor
        SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices , int source);
        
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
        
            vector<int> num_source; 
            vector<SOSP_Update> ssspTrees; 
        
            vector<vector<map<int, int>>> combinedGraph; 
            int numVertices;
        
        private:   
        
            vector<int> distances;
            vector<int> parent;
          //  void findUpdatedSOSPTree(const vector<tuple<int, int, int>>& insertions, int treeIndex);
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


SOSP_Update::SOSP_Update(const vector<vector<pair<int, int>>>& graph, int numVertices, int source)
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

    if (!distances.empty()) 
    {
        distances[source] = 0;  // Source node 
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
      parent(numVertices, -1) 
{
    for (size_t i = 0; i < sources.size(); ++i) 
    {
        ssspTrees.emplace_back(graph, numVertices, sources[i]); 
    }

    num_source = sources;
}

template <typename GraphType>
void printGraphList2(const GraphType& graph) {
    for (size_t u = 0; u < graph.size(); ++u) {
        cout << "V" << u + 1 << " -> ";
        for (const auto& edge : graph[u]) {
            cout << "V" << edge.first + 1 << " (edge = " << edge.second << ") ";
        }
        cout << endl;
    }
}

void MOSP_Update::update(const vector<tuple<int, int, int>>& insertions) 
{
    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    int treesPerProcess = ssspTrees.size() / size;
    int start = rank * treesPerProcess;
    int end = (rank == size - 1) ? ssspTrees.size() : start + treesPerProcess;

    vector<SOSP_Update> localTrees(ssspTrees.begin() + start, ssspTrees.begin() + end);

    if (rank == size - 1) 
    {
        cout << "Process " << rank << " has " << localTrees.size() << " trees." << endl;
    } 
    else 
    {
        cout << "Process " << rank << " has " << localTrees.size() << " trees." << endl;
    }

    MPI_Barrier(MPI_COMM_WORLD);

    for (int i = 0; i < localTrees.size(); ++i) 
    {
        cout << "Process " << rank << " updating tree "<<i <<" for source " << num_source[start + i] << endl;
        localTrees[i].update(insertions);
        localTrees[i].removeAffectedEdges();

        cout<< "Updated Tree "<<i<<endl;
        printGraphList2(localTrees[i].getGraph()); 

    }
    MPI_Barrier(MPI_COMM_WORLD);

    // Partial result from this process
    vector<vector<map<int, int>>> partialCombinedGraph(numVertices, vector<map<int, int>>(numVertices)); 
    for (const auto& sospTree : localTrees) 
    {
        const auto& graph = sospTree.getGraph();
        for (int u = 0; u < numVertices; ++u) 
        {
            for (const auto& edge : graph[u]) 
            {
                int v = edge.first;
                int weight = edge.second;
                partialCombinedGraph[u][v][weight]++;
            }
        }
    }

    // Serialize graph
    vector<int> serializedPartialGraph;
    for (int u = 0; u < numVertices; ++u) 
    {
        for (int v = 0; v < numVertices; ++v) 
        {
            for (const auto& [weight, count] : partialCombinedGraph[u][v]) 
            {
                serializedPartialGraph.push_back(u);
                serializedPartialGraph.push_back(v);
                serializedPartialGraph.push_back(weight);
                serializedPartialGraph.push_back(count);
            }
        }
    }

    // Gather all sizes
    int localSize = serializedPartialGraph.size();
    vector<int> allSizes(size);

    MPI_Barrier(MPI_COMM_WORLD); 

    MPI_Gather(&localSize, 1, MPI_INT, allSizes.data(), 1, MPI_INT, 0, MPI_COMM_WORLD);

    // Displacements for Gatherv
    vector<int> displacements(size, 0);
    int totalSize = 0;
    if (rank == 0) 
    {
        for (int i = 1; i < size; ++i) 
        {
            displacements[i] = displacements[i - 1] + allSizes[i - 1];
        }
        totalSize = displacements[size - 1] + allSizes[size - 1];
    }

    vector<int> globalSerializedGraph(totalSize);
    MPI_Gatherv(serializedPartialGraph.data(), localSize, MPI_INT,
                globalSerializedGraph.data(), allSizes.data(), displacements.data(), MPI_INT,
                0, MPI_COMM_WORLD);

    // Combine partial graphs on root
    if (rank == 0) 
    {
        combinedGraph.assign(numVertices, vector<map<int, int>>(numVertices));
        for (size_t i = 0; i < globalSerializedGraph.size(); i += 4) 
        {
            int u = globalSerializedGraph[i];
            int v = globalSerializedGraph[i + 1];
            int weight = globalSerializedGraph[i + 2];
            int count = globalSerializedGraph[i + 3];
            combinedGraph[u][v][weight] += count;

            if (count > 0) 
            {
                // Replace with meaningful weight (penalty logic)
                combinedGraph[u][v][1] = (num_source.size() - count + 1);
            }
        }

        findSOSPInCombinedGraph();
    }

    // Broadcast final result
    MPI_Bcast(distances.data(), distances.size(), MPI_INT, 0, MPI_COMM_WORLD);
    MPI_Bcast(parent.data(), parent.size(), MPI_INT, 0, MPI_COMM_WORLD);
}

const vector<int>& MOSP_Update::getDistances() const
{
    return distances;
}

const vector<int>& MOSP_Update::getParentArray() const 
{
    return parent;
}

void MOSP_Update::findSOSPInCombinedGraph() 
{
    fill(distances.begin(), distances.end(), numeric_limits<int>::max());
    fill(parent.begin(), parent.end(), -1);
    distances[0] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, 0});

    while (!pq.empty()) 
    {
        auto [dist, u] = pq.top();
        pq.pop();

        if (dist > distances[u]) continue;

        for (int v = 0; v < numVertices; ++v) 
        {
            for (const auto& [weight, _] : combinedGraph[u][v]) 
            {
                if (distances[v] > distances[u] + weight) 
                {
                    distances[v] = distances[u] + weight;
                    parent[v] = u;
                    pq.push({distances[v], v});
                }
            }
        }
    }
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


void printCombinedGraph(const vector<vector<map<int, int>>>& combinedGraph) {
    for (size_t u = 0; u < combinedGraph.size(); ++u) {
        cout << "Vertex " << u + 1 << " -> ";
        for (size_t v = 0; v < combinedGraph[u].size(); ++v) {
            if (!combinedGraph[u][v].empty()) {
                cout << "V" << v + 1 << " (weights: ";
                for (const auto& [weight, count] : combinedGraph[u][v]) {
                    cout << weight << " (count = " << count << "), ";
                }
                cout << ") ";
            }
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
    printCombinedGraph(mosp.combinedGraph);
}


int main(int argc, char** argv) 
{

    MPI_Init(&argc, &argv);

    int world_size;
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);

    int world_rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

    if (world_size < 2) 
    {
        if (world_rank == 0) 
        {
            cerr << "This program requires at least 2 MPI processes." << endl;
        }
      //  MPI_Finalize();
      //  return 1;
    }

    int numVertices = 600;
    vector<vector<pair<int, int>>> graph(numVertices);
    

    if (world_rank == 0) 
    {
        readGraph("graph.txt", graph);
    }

    // Broadcast the graph to all processes
    for (int i = 0; i < numVertices; ++i) {
        int size = graph[i].size();
        MPI_Bcast(&size, 1, MPI_INT, 0, MPI_COMM_WORLD);
        graph[i].resize(size);
        MPI_Bcast(graph[i].data(), size * sizeof(pair<int, int>), MPI_BYTE, 0, MPI_COMM_WORLD);
    }

    // Define sources and insertions
    vector<int> sources = {0, 0};
    vector<tuple<int, int, int>> insertions;

    if (world_rank == 0) {
        insertions = {
            {0, 1, 7},  // V1 -> V2 with weight 7
            {0, 4, 4},  // V1 -> V5 with weight 4
            {2, 4, 1}   // V3 -> V5 with weight 4
        };
    }

    // Broadcast insertions to all processes
    int numInsertions = insertions.size();
    MPI_Bcast(&numInsertions, 1, MPI_INT, 0, MPI_COMM_WORLD);
    insertions.resize(numInsertions);
    MPI_Bcast(insertions.data(), numInsertions * sizeof(tuple<int, int, int>), MPI_BYTE, 0, MPI_COMM_WORLD);

    if (world_rank == 0) {
        cout << "Graph initialized on process 0." << endl;
    }

    // Each process initializes its own MOSP_Update object
    MOSP_Update mosp(graph, sources);

    MPI_Barrier(MPI_COMM_WORLD);

    if (world_rank == 0) 
    {
        cout << "Graph broadcasted to all processes." << endl;
    }

    // Perform the update in parallel
    mosp.update(insertions);


    MPI_Barrier(MPI_COMM_WORLD);  // Ensure all processes have completed the update

    // Gather results on process 0 -- printing only 
    if (world_rank == 0)
    {
        cout << "Insertions done." << endl;

       
        cout << "Combined Graph: " << endl;
        printCombinedGraph(mosp.combinedGraph);

      //  cout << "Testing for large graph: " << endl;
     //   testLargeGraph();
    }

    MPI_Finalize();

    return 0;
}



