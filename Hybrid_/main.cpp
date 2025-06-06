#include <iostream>
#include <vector>
#include <mpi.h>
#include <queue>
#include <algorithm>
#include <limits>
#include <list>
#include <utility>
#include <fstream>
#include <algorithm>
#include <unordered_set>
#include <tuple>
#include <map>
#include <chrono>
using namespace std;
using namespace chrono;
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


// Open MP 
void SOSP_Update::updateParentAndDistance()
{
    #pragma omp parallel for
    for (int u = 0; u < numVertices; ++u) 
    {

        for (const auto& [v, w] : adjacencyList[u]) 
        {
            if (distances[u] != numeric_limits<int>::max() && distances[v] > distances[u] + w) 
            {
               
                #pragma omp critical
                {
                    if (distances[v] > distances[u] + w) 
                    {
                        distances[v] = distances[u] + w;
                        parent[v] = u;
                    }
                }
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


// Open MP 
void SOSP_Update::update(const vector<tuple<int, int, int>>& insertions)
{
    marked.assign(numVertices, 0);
    vector<int> Aff;

    
    #pragma omp parallel for
    for (size_t i = 0; i < insertions.size(); ++i)
    {
        const auto& [u, v, w] = insertions[i];

        #pragma omp critical
        {
            // Add to graph and reverse graph
            adjacencyList[u].emplace_back(v, w);
            reverseAdjList[v].emplace_back(u, w);
        }

        if (distances[u] != numeric_limits<int>::max() && distances[v] > distances[u] + w)
        {
            #pragma omp critical
            {
                if (distances[v] > distances[u] + w)
                {
                    distances[v] = distances[u] + w;
                    parent[v] = u;
                    if (!marked[v]) {
                        marked[v] = 1;
                        Aff.push_back(v);
                    }
                }
            }
        }
    }

    
//     // Step 1: Print the graph
//     cout << "Step 1 :" << endl;
//     for (int u = 0; u < numVertices; ++u) 
//     {
//         cout << "Node " << u << ":";
//         for (const auto& [v, w] : adjacencyList[u]) 
//         {
//             cout << " -> (" << v << ", " << w << ")";
//         }
//         cout << endl;
//     }
//     print the parent array
//     cout << "Parent Array before update: ";       
//     for (int i = 0; i < numVertices; ++i) {
//         cout << parent[i] << " ";
//     }cout << endl;

//    // updateParentAndDistance();

//     cout << "Parent Array after update: ";       
//     for (int i = 0; i < numVertices; ++i) {
//         cout << parent[i] << " ";
//     }cout << endl;

//     // Print Affected edges
//     cout << "Affected Edges: ";
//     for (int i = 0; i < Aff.size(); ++i) {
//         cout << Aff[i] << " ";
//     }cout <<endl;

    propagateUpdate(Aff);
}
// Open MP 
void SOSP_Update::propagateUpdate(vector<int>& Aff)
{
    vector<int> N;
        int m = 0; 
    
    while (!Aff.empty())
    {
        m++;
        vector<int> AffPrime;
        N.clear();
        
        #pragma omp parallel for
        for (int i = 0; i < Aff.size(); ++i)
        {
            int v = Aff[i];
            for (const auto& [nbr, _] : adjacencyList[v])
            {
                #pragma omp critical
                {
                    N.push_back(nbr);
                }
            }
        }

        #pragma omp parallel for
        for (int i = 0; i < N.size(); ++i)
        {
            int v = N[i];
            for (const auto& [u, w] : reverseAdjList[v])
            {
                if (!marked[u]) continue;

                if (distances[v] > distances[u] + w)
                {
                    distances[v] = distances[u] + w;
                    parent[v] = u;
                    if (!marked[v])
                     {
                        marked[v] = 1;
                        AffPrime.push_back(v);
                    }
                }
            }
        }

        Aff = move(AffPrime);
               // Print Affected edges
    // cout << "Affected Edges: at iteration "<< m << ": ";
    // for (int i = 0; i < Aff.size(); ++i) {
    //     cout << Aff[i] << " ";
    // } cout <<endl; 
     }

    updateParentAndDistance();
}
// Open MP 
void SOSP_Update::removeAffectedEdges()
{
    #pragma omp parallel for
    for (int u = 0; u < numVertices; ++u) {
        auto& edges = adjacencyList[u];
        for (auto it = edges.begin(); it != edges.end(); ) {
            int v = it->first;
            int w = it->second;

            bool shouldErase = false;
            #pragma omp critical
            {
                if (distances[v] != distances[u] + w || parent[v] != u) {
                    shouldErase = true;
                }
            }

            if (shouldErase) {
                #pragma omp critical
                {
                    it = edges.erase(it);
                }
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


//OpenMP
void MOSP_Update::update(const vector<tuple<int, int, int>>& insertions) 
{
    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);


    int treesPerProcess = max(1, static_cast<int>(ssspTrees.size() / size));
    int start = rank * treesPerProcess;

    int end = min(static_cast<int>(ssspTrees.size()), start + treesPerProcess);

    if (rank >= ssspTrees.size()) {
        start = end = 0; // Assign no data to processes exceeding the number of trees
    }

   // if (rank == 0) 
    {
        cout << "Process " << rank << " is responsible for trees from " << start << " to " << end - 1 << endl;
    }

    vector<SOSP_Update> localTrees(ssspTrees.begin() + start, ssspTrees.begin() + end);


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

        #pragma omp parallel for
        for (int u = 0; u < numVertices; ++u) 
        {
            for (const auto& edge : graph[u]) 
            {
                int v = edge.first;
                int weight = edge.second;

                #pragma omp critical
                {
                    partialCombinedGraph[u][v][weight]++;
                }
            }
        }
    }

    // Serialize graph
    vector<int> serializedPartialGraph;

    #pragma omp parallel for collapse(2)
    for (int u = 0; u < numVertices; ++u) 
    {
        for (int v = 0; v < numVertices; ++v) 
        {
            for (const auto& [weight, count] : partialCombinedGraph[u][v]) 
            {
                #pragma omp critical
                {
                    serializedPartialGraph.push_back(u);
                    serializedPartialGraph.push_back(v);
                    serializedPartialGraph.push_back(weight);
                    serializedPartialGraph.push_back(count);
                }
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
   // MPI_Bcast(distances.data(), distances.size(), MPI_INT, 0, MPI_COMM_WORLD);
  //  MPI_Bcast(parent.data(), parent.size(), MPI_INT, 0, MPI_COMM_WORLD);
}

const vector<int>& MOSP_Update::getDistances() const
{
    return distances;
}

const vector<int>& MOSP_Update::getParentArray() const 
{
    return parent;
}


//Open MP
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

        #pragma omp parallel for
        for (int v = 0; v < numVertices; ++v) 
        {
            for (const auto& [weight, _] : combinedGraph[u][v]) 
            {
            #pragma omp critical
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

void printCombinedGraph(const vector<vector<map<int, int>>>& combinedGraph, int numVertices) 
{
    
    
    for (size_t u = 0; u < numVertices; ++u) {
        cout << "Vertex " << u + 1 << " -> ";
        for (size_t v = 0; v < numVertices; ++v) 
        {
            if (!combinedGraph[u][v].empty()) 
            {
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



int main(int argc, char** argv) 
{
    auto start = high_resolution_clock::now();
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

    int numVertices = 6;
    vector<vector<pair<int, int>>> graph(numVertices);
    

    if (world_rank == 0) 
    {
    readGraph("small_dataset.txt", graph);
       
   
    }

    // Broadcast the graph to all processes
    for (int i = 0; i < numVertices; ++i)
     {
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
            {0, 4, 4},   // V1 -> V5 with weight 4
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
        printCombinedGraph(mosp.combinedGraph, numVertices);

      //  cout << "Testing for large graph: " << endl;
     //   testLargeGraph();
    }

    MPI_Finalize();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop-start);
    cout<<"Time taken using openMP is: "<<duration.count()<<endl;
    return 0;
}

// Small Dataset (8 threads, 2 processes): 292, 279, 494, 277 avg = 335.5 ms 
// Medium Dataset (8 threads, 2 processes): 477, 400, 392, 463 avg = 433 ms
// Large Dataset (8 threads, 2 processes): 536, 505, 520, 555 avg = 529 ms


// Small Dataset (4 threads, 2 processes): 292, 298, 285, 284 avg = 289.75 ms 
// Medium Dataset (4 threads, 2 processes): 373, 372, 375, 484 = 401 ms
// Large Dataset (4 threads, 2 processes): 467, 493, 470, 473 avg = 475.75

// Small Dataset (2 threads, 2 processes): 274, 311, 343, 312 avg = 310 ms 
// Medium Dataset (2 threads, 2 processes): 385, 368, 360, 391   = 376 ms
// Large Dataset (2 threads, 2 processes): 472, 844, 464, 802  avg = 645.5 ms

