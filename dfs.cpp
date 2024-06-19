//dfs 예제, 

#include <iostream>
#include <vector>
#include <stack>
using namespace std;

struct Point {
    int x, y;
};

int dfs(vector<vector<int>>& grid, int x, int y, vector<vector<bool>>& visited, int group_count) {
    stack<Point> stk;
    stk.push({x, y});
    int size = 0;

    int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    while (!stk.empty()) {
        Point p = stk.top();
        stk.pop();
        
        int cx = p.x, cy = p.y;
        
        if (visited[cx][cy]) continue;
        
        visited[cx][cy] = true;
        size++;
        grid[cx][cy] = group_count;
        
        for (auto& dir : directions) {
            int nx = cx + dir[0], ny = cy + dir[1];
            if (nx >= 0 && nx < grid.size() && ny >= 0 && ny < grid[0].size() && grid[nx][ny] == 1 && !visited[nx][ny]) {
                stk.push({nx, ny});
            }
        }
    }
    
    return size;
}

vector<vector<int>> mark_groups(vector<vector<int>>& grid) {
    int n = grid.size(), m = grid[0].size();
    vector<vector<bool>> visited(n, vector<bool>(m, false));
    int group_count = 2;  // 그룹 번호는 2부터 시작 (1은 하얀색, 0은 검은색)
    vector<int> group_sizes;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            if (grid[i][j] == 1 && !visited[i][j]) {
                int size = dfs(grid, i, j, visited, group_count);
                group_sizes.push_back(size);
                group_count++;
            }
        }
    }

    // 그룹 크기를 각 셀에 기록
    vector<vector<int>> result_grid(n, vector<int>(m, 0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            if (grid[i][j] > 1) {
                result_grid[i][j] = group_sizes[grid[i][j] - 2];
            }
        }
    }

    return result_grid;
}

int main() {
    vector<vector<int>> grid = {
        {1, 0, 0, 1},
        {1, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 1}
    };

    vector<vector<int>> result = mark_groups(grid);
    for (const auto& row : result) {
        for (int cell : row) {
            cout << cell << " ";
        }
        cout << endl;
    }

    return 0;
}
