// Compile: g++ -std=c++20 -pthread -O2 -o graph_query main.cpp
// Run: ./graph_query [graphFile] [queryFile] [numThreads]

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <queue>
#include <unordered_set>
#include <functional>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>

struct CSRGraph{
    int numVertices;
    std::vector<int> offsets;
    std::vector<int> edges;
};

CSRGraph LoadGraph(const std::string& filename){
    std::ifstream file(filename);
    std::string line;
    std::vector<std::pair<int, int>> edgeList;
    int maxVertex = -1;
    
    while(std::getline(file, line)){ 

        if(line[0] == '#') continue;    //ignore comment lines
        
        std::istringstream iss(line);
        int src, dst;
        
        if(iss >> src >> dst){
            edgeList.emplace_back(src, dst);
            maxVertex = std::max({maxVertex, src, dst});
        }

    }
    
    int numVertices = maxVertex + 1;
    CSRGraph graph; //initialize graph structure
    graph.numVertices = numVertices;
    graph.offsets.resize(numVertices + 1, 0);
    
    for(const auto& edge : edgeList){ //count outgoing edges for each vertex
        if(edge.first >= 0 && edge.first < numVertices){
            graph.offsets[edge.first + 1]++;
        }
    }
    
    for(int i = 0; i < numVertices; ++i){   //compute prefix sum to get offsets
        graph.offsets[i + 1] += graph.offsets[i];
    }
    
    std::vector<int> counters(numVertices, 0);
    graph.edges.resize(edgeList.size());
    for(const auto& edge : edgeList){
        if(edge.first >= 0 && edge.first < numVertices){    //place edges in the correct position
            int pos = graph.offsets[edge.first] + counters[edge.first]++;
            graph.edges[pos] = edge.second;
        }
    }
    
    return graph;
}

std::unordered_set<int> GetKHopReachable(const CSRGraph& graph, int src, int K){
    std::unordered_set<int> visited;
    
    if(src < 0 || src >= graph.numVertices || K <= 0){
        return visited;
    }
    
    std::queue<std::pair<int, int>> q;
    q.push({src, 0});
    visited.insert(src);
    
    while(!q.empty()){  //BFS up to K hops
        int curr = q.front().first;
        int depth = q.front().second;
        q.pop();
        
        if(depth == K) continue;
        
        for(int i = graph.offsets[curr]; i < graph.offsets[curr + 1]; ++i){
            int neighbor = graph.edges[i];
            if(visited.find(neighbor) == visited.end()){
                visited.insert(neighbor);
                q.push({neighbor, depth + 1});
            }
        }
    }
    
    visited.erase(src);
    return visited;
}

using QueryCallback = std::function<std::string(const CSRGraph&, int src, int K)>;

struct QueryTask{
    int src;
    int K;
    QueryCallback cb;
    std::string result;
};

std::string CountCallback(const CSRGraph& graph, int src, int K){   //count number of reachable vertices within K hops
    auto reachable = GetKHopReachable(graph, src, K);
    return std::to_string(reachable.size());
}

std::string MaxCallback(const CSRGraph& graph, int src, int K){ //find max vertex ID reachable within K hops
    auto reachable = GetKHopReachable(graph, src, K);
    if(reachable.empty()) return "-1";
    return std::to_string(*std::max_element(reachable.begin(), reachable.end()));
}

void RunTasksSequential(const CSRGraph& g, std::vector<QueryTask>& tasks){
    for(auto& task : tasks){
        task.result = task.cb(g, task.src, task.K);
    }
}

void RunTasksParallel(const CSRGraph& g, std::vector<QueryTask>& tasks, int numThreads){
    std::atomic<size_t> nextTask{0};
    std::vector<std::thread> workers;
    
    auto worker = [&g, &tasks, &nextTask](){
        while(true){
            size_t idx = nextTask.fetch_add(1);
            if(idx >= tasks.size()) break;
            tasks[idx].result = tasks[idx].cb(g, tasks[idx].src, tasks[idx].K);
        }
    };
    
    for(int i = 0; i < numThreads; ++i){
        workers.emplace_back(worker);
    }
    
    for(auto& t : workers){
        if(t.joinable()) t.join();
    }
}

std::vector<QueryTask> LoadQueries(const std::string& filename){
    std::vector<QueryTask> tasks;
    std::ifstream file(filename);
    std::string line;
    
    while(std::getline(file, line)){
        if(line[0] == '#') continue;
        
        std::istringstream iss(line);
        int src, K, queryType, expected;

        if(iss >> src >> K >> queryType >> expected){
            QueryTask task;
            task.src = src;
            task.K = K;
            task.cb =(queryType == 1) ? CountCallback : MaxCallback;
            tasks.push_back(task);
        }
    }
    return tasks;
}

int main(int argc, char* argv[]){
    std::string graphFile = "soc-Slashdot0902.txt";
    std::string queryFile = "queries20.txt";
    int numThreads = 4;
    
    if(argc > 1) graphFile = argv[1];
    if(argc > 2) queryFile = argv[2];
    if(argc > 3) numThreads = std::stoi(argv[3]);
    
    std::cout << "\nLoading graph: " << graphFile << "\n";
    CSRGraph graph = LoadGraph(graphFile);  //1.load graph
    
    std::cout << "Loading queries: " << queryFile << "\n";
    auto tasks = LoadQueries(queryFile);    //2.load queries and create tasks
    std::cout << "Loaded " << tasks.size() << " queries\n";
    
    auto sequentialTasks = tasks;
    auto t2 = std::chrono::high_resolution_clock::now();
    RunTasksSequential(graph, sequentialTasks); //run sequentially, record time
    auto t3 = std::chrono::high_resolution_clock::now();
    double sequentialTime = std::chrono::duration<double>(t3 - t2).count();
    std::cout << "Sequential time: " << sequentialTime << "s\n";
    
    auto parallelTasks = tasks;
    auto t4 = std::chrono::high_resolution_clock::now();
    RunTasksParallel(graph, parallelTasks, numThreads); //run concurrently, record time
    auto t5 = std::chrono::high_resolution_clock::now();
    double parallelTime = std::chrono::duration<double>(t5 - t4).count();
    std::cout << "Parallel time: " << parallelTime << "s\n";

    std::cout << "Speedup: " <<(parallelTime > 0 ? sequentialTime / parallelTime : 0) << "x\n";
    
    return 0;
}