// phase2.cpp
// g++ -std=c++20 -pthread -O2 -o phase2 phase2.cpp
// ./phase2 [graphFile] [numThreads]

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <algorithm>
#include <limits>
#include <chrono>
#include <tuple>

using namespace std;

struct CsrGraph{
    int numVertices;
    vector<int> offsets;
    vector<int> edges;
    vector<int> weights;
    vector<int> inOffsets;
    vector<int> inEdges;
    vector<int> inWeights;
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
    
    graph.inOffsets.resize(numVertices + 1, 0);
    for(int i = 0; i < edgeList.size(); i++){
        int dst = get<1>(edgeList[i]);
        graph.inOffsets[dst + 1]++;
    }
    for(int i = 0; i < numVertices; i++){
        graph.inOffsets[i + 1] += graph.inOffsets[i];
    }
    
    vector<int> fwdCnt(numVertices, 0), revCnt(numVertices, 0);
    graph.edges.resize(edgeList.size());
    graph.weights.resize(edgeList.size());
    graph.inEdges.resize(edgeList.size());
    graph.inWeights.resize(edgeList.size());
    
    for(int i = 0; i < edgeList.size(); i++){
        int src = get<0>(edgeList[i]);
        int dst = get<1>(edgeList[i]);
        int w = get<2>(edgeList[i]);
        
        int pos = graph.offsets[src] + fwdCnt[src]++;
        graph.edges[pos] = dst;
        graph.weights[pos] = w;
        
        pos = graph.inOffsets[dst] + revCnt[dst]++;
        graph.inEdges[pos] = src;
        graph.inWeights[pos] = w;
    }
    
    return graph;
}

class BspAlgorithm{
public:
    virtual bool HasWork() const = 0;
    virtual void Process(int tid, int v, CsrGraph& g) = 0;
    virtual void PostRound() = 0;
    virtual void PrintVertexResult(int v) = 0;
};

void BspSerial(CsrGraph& g, BspAlgorithm& algo){
    while(algo.HasWork()){
        for(int v = 0; v < g.numVertices; v++){
            algo.Process(0, v, g);
        }
        algo.PostRound();
    }
}

void BspParallel(CsrGraph& g, BspAlgorithm& algo, int numThreads){
    while(algo.HasWork()){
        vector<thread> workers;
        int chunk = g.numVertices / numThreads;
        
        for(int t = 0; t < numThreads; t++){
            int start = t * chunk;
            int end = (t + 1) * chunk;
            if(t == numThreads - 1) end = g.numVertices;
            
            workers.push_back(thread([&algo, &g, t, start, end](){
                for(int v = start; v < end; v++){
                    algo.Process(t, v, g);
                }
            }));
        }
        
        for(int i = 0; i < workers.size(); i++){
            workers[i].join();
        }
        algo.PostRound();
    }
}

class Bfs : public BspAlgorithm{
    int numVertices;
    int source;
    int numThreads;
    vector<int> distPrev;
    vector<int> distCurr;
    vector<int> threadUpdated;
    
public:
    Bfs(int n, int src, int nt){
        numVertices = n;
        source = src;
        numThreads = nt;
        distPrev.assign(n, -1);
        distCurr.assign(n, -1);
        distPrev[source] = 0;
        distCurr[source] = 0;
        threadUpdated.resize(nt, 0);
    }
    
    bool HasWork() const override{
        for(int i = 0; i < threadUpdated.size(); i++){
            if(threadUpdated[i]) return true;
        }
        return false;
    }
    
    void Process(int tid, int v, CsrGraph& g) override{
        if(v == source) return;
        int best = distCurr[v];
        for(int i = g.inOffsets[v]; i < g.inOffsets[v + 1]; i++){
            int u = g.inEdges[i];
            if(distPrev[u] != -1){
                int candidate = distPrev[u] + 1;
                if(best == -1 || candidate < best){
                    best = candidate;
                }
            }
        }
        if(best != distCurr[v]){
            distCurr[v] = best;
            threadUpdated[tid] = 1;
        }
    }
    
    void PostRound() override{
        distPrev = distCurr;
        for(int i = 0; i < threadUpdated.size(); i++){
            threadUpdated[i] = 0;
        }
    }
    
    void PrintVertexResult(int v) override{
        cout << "BFS:  source=" << source << " -> vertex=" << v 
             << ", hops = " << distCurr[v] << endl;
    }
};

class Sssp : public BspAlgorithm{
    int numVertices;
    int source;
    int numThreads;
    long long INF;
    vector<long long> distPrev;
    vector<long long> distCurr;
    vector<int> threadUpdated;
    
public:
    Sssp(int n, int src, int nt){
        numVertices = n;
        source = src;
        numThreads = nt;
        INF = numeric_limits<long long>::max() / 2;
        distPrev.assign(n, INF);
        distCurr.assign(n, INF);
        distPrev[source] = 0;
        distCurr[source] = 0;
        threadUpdated.resize(nt, 0);
    }
    
    bool HasWork() const override{
        for(int i = 0; i < threadUpdated.size(); i++){
            if(threadUpdated[i]) return true;
        }
        return false;
    }
    
    void Process(int tid, int v, CsrGraph& g) override{
        if(v == source) return;
        long long best = distCurr[v];
        for(int i = g.inOffsets[v]; i < g.inOffsets[v + 1]; i++){
            int u = g.inEdges[i];
            int w = g.inWeights[i];
            if(distPrev[u] != INF){
                long long candidate = distPrev[u] + w;
                if(candidate < best){
                    best = candidate;
                }
            }
        }
        if(best != distCurr[v]){
            distCurr[v] = best;
            threadUpdated[tid] = 1;
        }
    }
    
    void PostRound() override{
        distPrev = distCurr;
        for(int i = 0; i < threadUpdated.size(); i++){
            threadUpdated[i] = 0;
        }
    }
    
    void PrintVertexResult(int v) override{
        long long d = distCurr[v];
        if(d == INF) d = -1;
        cout << "SSSP: source=" << source << " -> vertex=" << v 
             << ", distance = " << d << endl;
    }
};

class Cc : public BspAlgorithm{
    int numVertices;
    int numThreads;
    vector<int> labelPrev;
    vector<int> labelCurr;
    vector<int> threadUpdated;
    
public:
    Cc(int n, int nt){
        numVertices = n;
        numThreads = nt;
        labelPrev.resize(n);
        labelCurr.resize(n);
        for(int i = 0; i < n; i++){
            labelPrev[i] = i;
            labelCurr[i] = i;
        }
        threadUpdated.resize(nt, 0);
    }
    
    bool HasWork() const override{
        for(int i = 0; i < threadUpdated.size(); i++){
            if(threadUpdated[i]) return true;
        }
        return false;
    }
    
    void Process(int tid, int v, CsrGraph& g) override{
        int best = labelCurr[v];
        for(int i = g.inOffsets[v]; i < g.inOffsets[v + 1]; i++){
            int u = g.inEdges[i];
            if(labelPrev[u] < best){
                best = labelPrev[u];
            }
        }
        for(int i = g.offsets[v]; i < g.offsets[v + 1]; i++){
            int u = g.edges[i];
            if(labelPrev[u] < best){
                best = labelPrev[u];
            }
        }
        if(best != labelCurr[v]){
            labelCurr[v] = best;
            threadUpdated[tid] = 1;
        }
    }
    
    void PostRound() override{
        labelPrev = labelCurr;
        for(int i = 0; i < threadUpdated.size(); i++){
            threadUpdated[i] = 0;
        }
    }
    
    void PrintVertexResult(int v) override{
        cout << "CC:   vertex=" << v << ", component = " << labelCurr[v] << endl;
    }
};

int main(int argc, char* argv[]){
    string graphFile = "soc-LiveJournal1-weighted.txt";
    int numThreads = 4;
    
    if(argc > 1) graphFile = argv[1];
    if(argc > 2) numThreads = atoi(argv[2]);
    
    auto t0 = chrono::high_resolution_clock::now();
    CsrGraph graph = LoadGraph(graphFile);
    auto t1 = chrono::high_resolution_clock::now();
    double loadTime = chrono::duration<double>(t1 - t0).count();

    
    int source = 0;
    int target = 50;
    
    cout << "-- BFS --" << endl;
    Bfs bfsSerial(graph.numVertices, source, 1);
    auto t2 = chrono::high_resolution_clock::now();
    BspSerial(graph, bfsSerial);
    auto t3 = chrono::high_resolution_clock::now();
    double bfsSerialTime = chrono::duration<double>(t3 - t2).count();
    
    Bfs bfsParallel(graph.numVertices, source, numThreads);
    auto t4 = chrono::high_resolution_clock::now();
    BspParallel(graph, bfsParallel, numThreads);
    auto t5 = chrono::high_resolution_clock::now();
    double bfsParallelTime = chrono::duration<double>(t5 - t4).count();
    
    bfsParallel.PrintVertexResult(target);
    cout << "BFS serial: " << bfsSerialTime << "s, parallel: " 
         << bfsParallelTime << "s, speedup: " 
         << (bfsParallelTime > 0 ? bfsSerialTime / bfsParallelTime : 0) << "x" << endl;
    
    cout << "-- SSSP --" << endl;
    Sssp ssspSerial(graph.numVertices, source, 1);
    auto t6 = chrono::high_resolution_clock::now();
    BspSerial(graph, ssspSerial);
    auto t7 = chrono::high_resolution_clock::now();
    double ssspSerialTime = chrono::duration<double>(t7 - t6).count();
    
    Sssp ssspParallel(graph.numVertices, source, numThreads);
    auto t8 = chrono::high_resolution_clock::now();
    BspParallel(graph, ssspParallel, numThreads);
    auto t9 = chrono::high_resolution_clock::now();
    double ssspParallelTime = chrono::duration<double>(t9 - t8).count();
    
    ssspParallel.PrintVertexResult(target);
    cout << "SSSP serial: " << ssspSerialTime << "s, parallel: " 
         << ssspParallelTime << "s, speedup: " 
         << (ssspParallelTime > 0 ? ssspSerialTime / ssspParallelTime : 0) << "x" << endl;
    
    cout << "-- CC --" << endl;
    Cc ccSerial(graph.numVertices, 1);
    auto t10 = chrono::high_resolution_clock::now();
    BspSerial(graph, ccSerial);
    auto t11 = chrono::high_resolution_clock::now();
    double ccSerialTime = chrono::duration<double>(t11 - t10).count();
    
    Cc ccParallel(graph.numVertices, numThreads);
    auto t12 = chrono::high_resolution_clock::now();
    BspParallel(graph, ccParallel, numThreads);
    auto t13 = chrono::high_resolution_clock::now();
    double ccParallelTime = chrono::duration<double>(t13 - t12).count();
    
    ccParallel.PrintVertexResult(target);
    cout << "CC serial: " << ccSerialTime << "s, parallel: " 
         << ccParallelTime << "s, speedup: " 
         << (ccParallelTime > 0 ? ccSerialTime / ccParallelTime : 0) << "x" << endl;
    
    return 0;
}