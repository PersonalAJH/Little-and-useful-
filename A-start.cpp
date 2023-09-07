#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>


/*
디익스트라 알고리즘과 A*알고리즘의 차이 
https://youtu.be/RYdBcnSiwag?si=1mYl_xonRgJ-Cs5c

예시가 유튜브에 잘나와있음 

디익스트라 알고리즘의 경우 휴리스틱 함수의 비교가 없기 때문에 방향성을 갖지 못해 시작점으로 부터 원형으로 탐색을 하며(시작점으로 부터 거리가 가장 가까운곳)
A*알고리즘의 경우 방향성을 가지기 때문에 디익스트라 알고리즘보다 훨씬더 빠르게 길을 찾을 수 있다. 만약에 휴리스틱 함수의 weight? value가 모두 같다면 A* 알고리즘과 디익스트라 알고리즘은 같아지게 된다. 

*/

struct Node {
    int x, y; // Node의 좌표
    double g; // 시작 노드로부터의 비용
    double h; // 목표 노드로부터의 예상 비용
    double f; // f = g + h
    Node* parent; // 이전 노드에 대한 포인터

    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}

    // 비용 계산을 위한 휴리스틱 함수 -> 대표적으로는 맨해튼, 유클리디안이 있는데 여기서는 맨해튼(계산이 편함)
    double calculateHeuristic(const Node& goal) {
        return std::abs(x - goal.x) + std::abs(y - goal.y);
    }
};

// 우선순위 큐를 사용하여 노드를 정렬
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;
    }
};

std::vector<Node*> aStarSearch(Node* start, Node* goal) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openSet;
    std::map<std::pair<int, int>, Node*> nodeMap;

    openSet.push(start);
    nodeMap[std::make_pair(start->x, start->y)] = start;

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current == goal) {
            // 목표에 도달한 경우 경로를 복원
            std::vector<Node*> path;
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 이웃 노드 순회
        std::vector<std::pair<int, int>> neighbors = {
            {current->x + 1, current->y},
            {current->x - 1, current->y},
            {current->x, current->y + 1},
            {current->x, current->y - 1}
        };

        for (const auto& neighborPos : neighbors) {
            int nx = neighborPos.first;
            int ny = neighborPos.second;

            if (nx < 0 || ny < 0) continue; // 범위를 벗어나면 무시

            // 새로운 이웃 노드 생성
            Node* neighbor = new Node(nx, ny);
            neighbor->g = current->g + 1; // currnet 에서 neighbor 롤 움직이는 weight -> 여기서는 1로 고정
            neighbor->h = neighbor->calculateHeuristic(*goal);
            neighbor->f = neighbor->g + neighbor->h;
            neighbor->parent = current;

            // 이미 방문한 노드인지 확인
            if (nodeMap.find(std::make_pair(nx, ny)) != nodeMap.end()) {
                Node* existingNeighbor = nodeMap[std::make_pair(nx, ny)];
                if (neighbor->f >= existingNeighbor->f) {
                    delete neighbor;
                    continue;
                }
            }

            openSet.push(neighbor);
            nodeMap[std::make_pair(nx, ny)] = neighbor;
        }
    }

    // 목표에 도달할 수 없는 경우 빈 벡터 반환
    return std::vector<Node*>();
}

int main() {
    Node start(0, 0);
    Node goal(5, 5);

    std::vector<Node*> path = aStarSearch(&start, &goal);

    if (!path.empty()) {
        std::cout << "최단 경로: ";
        for (Node* node : path) {
            std::cout << "(" << node->x << ", " << node->y << ") -> ";
        }
        std::cout << "도착!" << std::endl;
    } else {
        std::cout << "목표에 도달할 수 없습니다." << std::endl;
    }

    // 메모리 해제
    for (Node* node : path) {
        delete node;
    }

    return 0;
}


