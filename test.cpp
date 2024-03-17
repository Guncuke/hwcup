#include<bits/stdc++.h>

using namespace std;
const int N = 210;
const int n = 200;
const int robot_num = 10;
char ch[N][N];

struct Item
{
	// 物品位置，价值和剩余帧数
	int x, y, val, appear_frame;
	Item(){}
	Item(int x, int y, int val, int appear_frame) {
		this -> x = x;
		this -> y = y;
		this -> val = val;
		this -> appear_frame = appear_frame;
	}
	// 重载运算符，从大到小排序
	bool operator < (const Item &a) const {
		return val > a.val;
	}
};

Item items = Item(137, 117, 187, 1);
struct Robot
{
	int x, y, goods;
	int status;
	int mbx, mby;
	Robot() {}
	Robot(int startX, int startY) {
		x = startX;
		y = startY;
	}
}robot[robot_num + 10];

// 定义节点结构体
struct Node {
	int x, y;
	int g;  // 从起点到当前节点的实际距离
	int h;  // 从当前节点到目标节点的启发式距离
	pair<int, int> parent;  // 用来获得路径
	bool operator<(const Node& rhs) const {
		return g + h > rhs.g + rhs.h;
	}
};
// 计算启发式距离
int Heuristic(int x, int y, int xx, int yy)
{
	return abs(x - xx) + abs(y - yy);
}

// 十个机器人的路径
list<int> paths[10];
list<int>::iterator current_index[10];
// A*搜索算法
void AStarSearch(Item target, int robot_index) {
	priority_queue<Node> open_list;
	bool closed_list[n][n] = {false};
	// 用来获得路径
	Node parents[n][n];
	for(int i = 0; i < n; i++) 
	{
		for(int j = 0; j < n; j++) {
			parents[i][j] = {-1, -1};
		}
	}
	int directions[n][n];
	int start_x = robot[robot_index].x, start_y = robot[robot_index].y;
	open_list.push({start_x, start_y, 0, Heuristic(start_x, start_y, target.x, target.y), {-1, -1}});
	
	while (!open_list.empty()) {
		Node current = open_list.top();
		printf("current.x = %d, current.y = %d\n", current.x, current.y);
		open_list.pop();
		
		if (current.x == target.x && current.y == target.y) {
			// 保存路径
			for (Node p = current; p.x != -1 && p.y != -1; p = parents[p.x][p.y]) {
				printf("%c", ch[p.x][p.y]);
				paths[robot_index].push_front(directions[p.x][p.y]);
			}
			printf("\n");
			paths[robot_index].pop_front();
			return;
		}
		
		if (closed_list[current.x][current.y]) {
			continue;  // 已经检查过这个节点
		}
		closed_list[current.x][current.y] = true;
		
		// 检查周围的节点
		int dxs[] = {0, 0, -1, 1};
		int dys[] = {1, -1, 0, 0};
		for (int i = 0; i < 4; i++) {
			int nx = current.x + dxs[i];
			int ny = current.y + dys[i];
			if (nx >= 0 && nx < 200 && ny >= 0 && ny < 200 && !closed_list[nx][ny] && ch[nx][ny] == '.') {
				parents[nx][ny] = current;
				directions[nx][ny] = i;
        		open_list.push({nx, ny, current.g + 1, Heuristic(nx, ny, target.x, target.y), {current.x, current.y}});
			}
		}
	}
}

int main(){
	robot[0].x = 36;
	robot[0].y = 173;
	// 从output.txt读入前20行地图到ch
	ifstream fin("output.txt");
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			fin >> ch[i][j];
		}
	}
	
	AStarSearch(items, 0);
	// 输出path[0]
	int cur_x = robot[0].x, cur_y = robot[0].y;
	int dxs[] = {0, 0, -1, 1};
	int dys[] = {1, -1, 0, 0};
	printf("%c", ch[cur_x][cur_y]);
	for (auto it = paths[0].begin(); it != paths[0].end(); it++) {
		cur_x += dxs[*it];
		cur_y += dys[*it];
		printf("%c", ch[cur_x][cur_y]);
	}
	return 0;
}
