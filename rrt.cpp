#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <SFML/Graphics.hpp>

using namespace std;
// 2차원 좌표축 -> ROS message
struct Point {
    double x;
    double y;
};

// RRT 노드
struct Node {
    Point point;
    int parentIdx;
};

// 랜덤 포인트 생성 
Point getRandomPoint(double minX, double maxX, double minY, double maxY) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_real_distribution<double> xDist(minX, maxX);
    uniform_real_distribution<double> yDist(minY, maxY);
    Point randomPoint;
    randomPoint.x = xDist(gen);
    randomPoint.y = yDist(gen);
    return randomPoint;
}

//  가장 가까운 노드 확인
int getNearestNode(const vector<Node>& tree, const Point& randomPoint) {
    double minDist = numeric_limits<double>::max();
    int nearestIdx = -1;
    
    for (int i = 0; i < tree.size(); ++i) {
        double dist = hypot(tree[i].point.x - randomPoint.x, tree[i].point.y - randomPoint.y);
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }
    
    return nearestIdx;
}


Node extendTree(const vector<Node>& tree, const Point& randomPoint, double maxStep) {
    int nearestIdx = getNearestNode(tree, randomPoint);
    Point nearestPoint = tree[nearestIdx].point;
    double dist = hypot(nearestPoint.x - randomPoint.x, nearestPoint.y - randomPoint.y);
    
    // 랜덤 point 와 거리 비교 후 이동 거리가 max step 보다 작다면 그냥 이동하지만 아니라면 max step만큼만 이동(가야하는 point의 위치와 방향을 알기 떄문)
    if (dist <= maxStep) {
        return {randomPoint, nearestIdx};
    } else {
        double fraction = maxStep / dist;
        double newX = nearestPoint.x + fraction * (randomPoint.x - nearestPoint.x);
        double newY = nearestPoint.y + fraction * (randomPoint.y - nearestPoint.y);
        return {{newX, newY}, nearestIdx};
    }
}

// point 간의 line에서 collision이 없는지 확인(방해물)
bool isSegmentValid(const Point& start, const Point& end, const vector<sf::RectangleShape>& obstacles) {
    for (const sf::RectangleShape& obstacle : obstacles) {
        sf::FloatRect obstacleBounds = obstacle.getGlobalBounds();
        
        Point p1 = {obstacleBounds.left, obstacleBounds.top};
        Point p2 = {obstacleBounds.left + obstacleBounds.width, obstacleBounds.top};
        
        if (std::hypot(start.x - end.x, start.y - end.y) <= 0.01) {
            continue;  
        }
        
        double det = (p2.x - p1.x) * (end.y - start.y) - (end.x - start.x) * (p2.y - p1.y);
        
        if (det == 0) {
            continue;  
        }
        
        double t = ((end.x - start.x) * (start.y - p1.y) - (end.y - start.y) * (start.x - p1.x)) / det;
        double u = -((p2.x - p1.x) * (start.y - p1.y) - (p2.y - p1.y) * (start.x - p1.x)) / det;
        
        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
            return false; 
        }
    }
    
    return true;  
}


std::vector<Point> findPath(const Point& startPoint, const Point& goalPoint, const std::vector<sf::RectangleShape>& obstacles, double maxStep, int maxIterations) {
    std::vector<Node> tree = {{startPoint, -1}};
    
    for (int i = 0; i < maxIterations; ++i) {
        Point randomPoint = getRandomPoint(0, 800, 0, 600); 
        Node newNode = extendTree(tree, randomPoint, maxStep);
        
        if (isSegmentValid(tree[newNode.parentIdx].point, newNode.point, obstacles
