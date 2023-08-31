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

int main() {
    int n = 4; // 노드 수
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
