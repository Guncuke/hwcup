#include <bits/stdc++.h>
using namespace std;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;

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

struct Berth
{   
    // 泊位左上角位置，泊位大小4*4
    int x;
    int y;
    // 泊位到虚拟点需要的时间
    int transport_time;
    // 泊位装载速度
    int loading_speed;
    Berth(){}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this -> x = x;
        this -> y = y;
        this -> transport_time = transport_time;
        this -> loading_speed = loading_speed;
    }
}berth[berth_num + 10];

struct Boat
{   
    // num: 当前装载量
    // status: 0: 运输中 1: 装货或运输完成 2:泊位外等待
    // pos: 目标泊位
    int num, pos, status;
    Boat(){}
    Boat(int num, int pos, int status) {
        this -> num = num;
        this -> pos = pos;
        this -> status = status;
    }
}boat[boat_num + 10];

struct Item
{
    // 物品位置，价值和剩余帧数
    int x, y, val, surplus_time;
    Item(){}
    Item(int x, int y, int val) {
        this -> x = x;
        this -> y = y;
        this -> val = val;
    }
    // 重载运算符，从大到小排序
    bool operator < (const Item &a) const {
        return val > a.val;
    }
};

int money, boat_capacity, id;
char ch[N][N];
int gds[N][N];
void Init()
{
    for(int i = 1; i <= n; i ++)
        scanf("%s", ch[i]);
    for(int i = 0; i < berth_num; i ++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}
// 物品优先队列，价值越大越靠前
priority_queue<Item> items;

int Input()
{
    // 当前帧和金钱
    scanf("%d%d", &id, &money);
    int num;
    // 场上新增货物数量
    scanf("%d", &num);
    for(int i = 1; i <= num; i ++)
    {
        // 新增物品的位置和价值
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        items.push(Item(x, y, val));
    }

    for(int i = 0; i < robot_num; i ++)
    {
        int sts;
        // 是否携带货物，坐标，状态
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
    }
    for(int i = 0; i < boat_num; i ++)
        // 船的状态和目标泊位
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    char okk[100];
    scanf("%s", okk);
    puts("OK");
    return id;
}

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
list<int>::iterator current[10];
// A*搜索算法
void AStarSearch(Item target, int robot_index) {
    priority_queue<Node> open_list;
    bool closed_list[n][n] = {false};
    // 用来获得路径
    Node parents[n][n];
    int directions[n][n];

    int start_x = robot[robot_index].x, start_y = robot[robot_index].y;
    open_list.push({start_x, start_y, 0, Heuristic(start_x, start_y, target.x, target.y), {-1, -1}});

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (current.x == target.x && current.y == target.y) {
            // 保存路径
            for (Node p = current; p.x != -1 && p.y != -1; p = parents[p.x][p.y]) {
                paths[robot_index].push_front(directions[p.x][p.y]);
            }
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
                open_list.push({nx, ny, current.g + 1, Heuristic(nx, ny, target.x, target.y), {current.x, current.y}});
                parents[nx][ny] = current;
                directions[nx][ny] = i;
            }
        }
    }
}

int main()
{
    Init();
    for (int i = 0; i < 10; ++i) 
    {
        current[i] = paths[i].begin();
    }
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
        int id = Input();
        if(zhen == 1) {
            AStarSearch(items.top(), 0);
        }
        // if(robot[0].goods == 0 && !items.empty() && paths[0].empty()) {
        //     AStarSearch(items.top(), 0);
        // }
        if(!paths[0].empty() and current[0] != paths[0].end()) {
            printf("move %d %d", 0, *current[0]);
            current[0]++;
        }
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
