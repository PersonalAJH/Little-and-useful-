#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

typedef pair<int, int> pair_int;

void dijkstra(vector<vector<pair_int>>& graph, int start) {
    int n = graph.size();
    //출발점이 정해져 있으니까-> 내가 원하는 것만
    vector<int> distances(n, INT_MAX);
    distances[start] = 0;
    priority_queue<pair_int, vector<pair_int>, greater<pair_int>> pq;

    
    pq.push(make_pair(0, start));
    while (!pq.empty()) {
        int current_distance = pq.top().first;
        int current_node = pq.top().second;
        pq.pop();

        if (current_distance > distances[current_node])
            continue;

        for (const pair_int& neighbor : graph[current_node]) {
            int neighbor_node = neighbor.first;
            int weight = neighbor.second;

            if (current_distance + weight < distances[neighbor_node]) {
                distances[neighbor_node] = current_distance + weight;
                pq.push(make_pair(distances[neighbor_node], neighbor_node));
            }
        }
    }

    // 최단 거리 출력
    for (int i = 1; i < n; ++i) {
        cout << "노드 " << i << "까지의 최단 거리: " << distances[i] << endl;
    }
}


int getSmallIndex(vector<int> dis){

    int min = INT_MAX;
    int index;

    for(int i = 0; i < dis.size(); i++){
        if(min < dis.at(i)){
            min = dis.at(i);
            index = i ;
        }
    }
    


    return index;
}



void dijkstra2(vector<vector<pair_int>>& graph, int start) {
    int n = graph.size();
    //출발점이 정해져 있으니까-> 내가 원하는 것만
    vector<int> distances(n, INT_MAX);
    distances[start] = 0;
    priority_queue<pair_int, vector<pair_int>, greater<pair_int>> pq;

    //start num 에 있을 때 distances 를 구하기
    //distances = [0,1,4,MAX]
    for(int i = 0; i < graph[start].size(); i++){
        if(distances.at(i) < graph[start].at(i).second){
            distances.at(i) = graph[start].at(i).second;
        }
    }

    // weight가 가장 작은 노드로 넘어가서 distances를 업데이트하기
    for(int i = 0; i < graph.size(); i++){
        for(auto j : graph.at(i)){
            pq.push(j);
        }

    }


    // 그 다음 weight로 가서 distances를 업데이트하기 

    





    // 최단 거리 출력
    for (int i = 1; i < n; ++i) {
        cout << "노드 " << i << "까지의 최단 거리: " << distances[i] << endl;
    }
}




int main() {
    int n = 5; // 노드 수
    vector<vector<pair_int>> graph(n);

    graph[1].push_back(make_pair(2, 1));
    graph[1].push_back(make_pair(3, 4));
    graph[2].push_back(make_pair(1, 1));
    graph[2].push_back(make_pair(3, 2));
    graph[2].push_back(make_pair(4, 5));
    graph[3].push_back(make_pair(1, 4));
    graph[3].push_back(make_pair(2, 2));
    graph[3].push_back(make_pair(4, 1));
    graph[4].push_back(make_pair(2, 5));
    graph[4].push_back(make_pair(3, 1));

    int start_node = 1;
    dijkstra(graph, start_node);

    return 0;
}
