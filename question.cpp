#include <string>
#include <vector>
#include <queue>

using namespace std;

int solution(int x, int y, int n) {
    queue<pair<int, int>> q;
    q.push(make_pair(y, 0));

    while (!q.empty())
        {
            int val = q.front().first;
            int cnt_ = q.front().second;
            q.pop();
            printf("size : %d \n",q.size()); 
            
            if (val == x)
                return cnt_;

            if (val % 2 == 0)
            {
                q.push(make_pair(val / 2, cnt_ + 1)); 
            }

            if (val % 3 == 0)
            {
                q.push(make_pair(val / 3, cnt_ + 1));
            }

            if (val - n > 0)
            {
                q.push(make_pair(val - n, cnt_ + 1));
            }
        }
    return -1;
}
