#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;


/*
//설명 대강 
// 다이나믹 프로그래밍을 이용한 최단 경로 탐색 알고리즘 
흔히 인공위성 GPS 소프트웨어 등을 많이 사용한다. 특정한 하나의 정점과 다른 모든 정점으로 가는 최단 경로를 알려준다. 


정확성: 다익스트라 알고리즘은 가중치가 음수가 아닌 경우에 정확한 최단 경로를 찾습니다.
유연성: 다양한 종류의 그래프에서 사용할 수 있습니다. 방향성, 무방향성, 가중치 등 다양한 형태의 그래프에서 적용 가능합니다.
단일 출발지: 하나의 출발 노드에서 다른 모든 노드까지의 최단 경로를 찾는 데 효과적입니다.
간선 중복 허용: 그래프 내에서 간선의 중복을 허용하는 경우에도 정확한 결과를 제공합니다.
데이터 구조 활용: 우선순위 큐를 사용하여 구현할 수 있어, 효율적인 데이터 구조를 활용하여 빠른 실행이 가능합니다.
단점:

음수 가중치 처리 어려움: 다익스트라 알고리즘은 음수 가중치를 처리하는 데 제약이 있습니다. 음수 가중치가 있는 경우에는 벨만-포드 알고리즘과 같은 다른 알고리즘이 필요합니다.
최단 경로 갱신 문제: 다익스트라 알고리즘은 최단 경로를 찾은 노드의 가중치를 갱신하는 과정에서 오류가 발생할 수 있습니다. 이를 보완하기 위해 큐를 활용하는데, 이는 코드 구현에 복잡성을 추가할 수 있습니다.
Dense 그래프에서 성능 저하: 그래프가 밀집되어있는 경우(간선이 많은 경우) 성능이 저하될 수 있습니다. 이런 경우에는 힙 대신에 배열 기반의 자료 구조를 사용하여 성능 개선을 시도할 수 있습니다.
완전 그래프에서 성능 저하: 그래프가 완전 그래프 형태일 경우(모든 노드가 서로 연결된 경우) 효율성이 떨어질 수 있습니다.



*/
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
            index = i;
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
    // 첫번째 distances = [0,1,4,MAX]
    // 두번째 distances 는 grapg.at(t)로 들어가서 더하고 작으면 graph2 에서 (1,1),(3,2),(4,5)
    int t;
    t = getSmallIndex(distances);
    graph.at(t);
    for(int i = 0; i < graph[t].size(); i++){
        int tmp_node = graph[t].at(i).first;
        int tmp_weight = graph[t].at(i).second;
        if(distances.at(t) + tmp_weight < distances.at(tmp_node)){
            distances.at(tmp_node) = distances.at(t) + tmp_weight;
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
