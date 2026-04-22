#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>

using namespace std;

struct CsrGraph{
    int numVertices;
    vector<int> offsets;
    vector<int> edges;
    vector<int> weights;
};

CsrGraph LoadGraph(string filename){
    ifstream file(filename);
    string line;
    vector<tuple<int, int, int>> edgeList;
    int maxVertex = -1;
    
    while(getline(file, line)){
        if(line.empty() || line[0] == '#') continue;
        istringstream iss(line);
        int src, dst, weight;
        iss >> src >> dst >> weight;
        edgeList.push_back(make_tuple(src, dst, weight));
        if(src > maxVertex) maxVertex = src;
        if(dst > maxVertex) maxVertex = dst;
    }
    
    int numVertices = maxVertex + 1;
    CsrGraph graph;
    graph.numVertices = numVertices;
    
    graph.offsets.resize(numVertices + 1, 0);
    for(int i = 0; i < edgeList.size(); i++){
        int src = get<0>(edgeList[i]);
        graph.offsets[src + 1]++;
    }
    for(int i = 0; i < numVertices; i++){
        graph.offsets[i + 1] += graph.offsets[i];
    }
    
    vector<int> counters(numVertices, 0);
    graph.edges.resize(edgeList.size());
    graph.weights.resize(edgeList.size());
    for(int i = 0; i < edgeList.size(); i++){
        int src = get<0>(edgeList[i]);
        int dst = get<1>(edgeList[i]);
        int w = get<2>(edgeList[i]);
        int pos = graph.offsets[src] + counters[src]++;
        graph.edges[pos] = dst;
        graph.weights[pos] = w;
    }
    
    return graph;
}

class BfsPushNaive{
    int numVertices;
    int source;
    vector<int> dist;
    vector<int> distPrev;
    bool anyUpdated;
    
public:
    BfsPushNaive(int n, int src){
        numVertices = n;
        source = src;
        dist.assign(n, -1);
        distPrev.assign(n, -1);
        dist[source] = 0;
        distPrev[source] = 0;
        anyUpdated = true;
    }
    
    bool HasWork(){
        return anyUpdated;
    }
    
    void Process(int u, CsrGraph& g){
        if(distPrev[u] == -1) return; //skip unknown vertices
        
        for(int i = g.offsets[u]; i < g.offsets[u + 1]; i++){ //push to neighbors
            int v = g.edges[i];
        
            if(dist[v] == -1){ //set dist and mark updated
                dist[v] = distPrev[u] + 1;
            }
        }
    }
    
    void PostRound(){ //check for chanegs
        anyUpdated = false;
        for(int v = 0; v < numVertices; v++){
            if(dist[v] != distPrev[v]){
                anyUpdated = true;
            }
        }

        distPrev = dist;
    }
    
    int GetDistance(int v){
        return dist[v];
    }
    
    void Run(CsrGraph& g){
        while(HasWork()){
            for(int u = 0; u < numVertices; u++){
                Process(u, g);
            }
            PostRound();
        }
    }
};

class BfsPushDense{
    int numVertices;
    int source;
    vector<int> dist;
    vector<int> distPrev;
    vector<uint8_t> inFrontier;
    vector<uint8_t> inNext;
    bool anyUpdated;
    
public:
    BfsPushDense(int n, int src){
        numVertices = n;
        source = src;
        dist.assign(n, -1);
        distPrev.assign(n, -1);
        inFrontier.assign(n, 0);
        inNext.assign(n, 0);
        
        dist[source] = 0;
        distPrev[source] = 0;
        inFrontier[source] = 1;
        anyUpdated = true;
    }
    
    bool HasWork(){
        return anyUpdated;
    }
    
    void Process(int u, CsrGraph& g){
        if(!inFrontier[u]) return;  //skip vertices not in frontier
        if(distPrev[u] == -1) return;
        
        for(int i = g.offsets[u]; i < g.offsets[u + 1]; i++){
            int v = g.edges[i];
            if(dist[v] == -1){ //set dist and mark updated
                dist[v] = distPrev[u] + 1;
                inNext[v] = 1;
            }
        }
    }
    
    void PostRound(){ //check for changes and prepare next frontier
        anyUpdated = false;
        for(int v = 0; v < numVertices; v++){
            if(dist[v] != distPrev[v]){
                anyUpdated = true;
            }
        }
        distPrev = dist;
        inFrontier = inNext;
        fill(inNext.begin(), inNext.end(), 0);
    }
    
    int GetDistance(int v){
        return dist[v];
    }
    
    void Run(CsrGraph& g){
        while(HasWork()){
            for(int u = 0; u < numVertices; u++){
                Process(u, g);
            }
            PostRound();
        }
    }
};

int main(int argc, char* argv[]){
    string graphFile = "soc-LiveJournal1-weighted.txt";
    if(argc > 1) graphFile = argv[1];
    CsrGraph graph = LoadGraph(graphFile);
    
    int source = 0;
    int target = 50;
    
    cout << "Running BFS (naive push)" << endl;     //naive push BFS
    auto t2 = chrono::high_resolution_clock::now();
    BfsPushNaive bfsNaive(graph.numVertices, source);
    bfsNaive.Run(graph);
    auto t3 = chrono::high_resolution_clock::now();
    double naiveTime = chrono::duration<double>(t3 - t2).count();
    
    cout << "Running BFS (dense worklist)" << endl;     //dense worklist BFS
    auto t4 = chrono::high_resolution_clock::now();
    BfsPushDense bfsDense(graph.numVertices, source);
    bfsDense.Run(graph);
    auto t5 = chrono::high_resolution_clock::now();
    double denseTime = chrono::duration<double>(t5 - t4).count();
    
    cout << endl;
    cout << "BFS (naive push): source=" << source << " -> vertex=" << target 
         << ", hops = " << bfsNaive.GetDistance(target) << endl;
    cout << "BFS (dense worklist): source=" << source << " -> vertex=" << target 
         << ", hops = " << bfsDense.GetDistance(target) << endl;
    
    cout << endl;   //timing
    cout << "Naive push time: " << naiveTime << "s" << endl;
    cout << "Dense worklist time: " << denseTime << "s" << endl;

    return 0;
}