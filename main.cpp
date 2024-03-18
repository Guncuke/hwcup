#include <bits/stdc++.h>
#include<fstream>
using namespace std;
fstream f1;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;
// 金钱，船只容量，当前帧数
int money, boat_capacity, id;
const int dxs[] = {0, 0, -1, 1};
const int dys[] = {1, -1, 0, 0};
// 地图
char ch[N][N];
int gds[N][N];
// 十个机器人的路径
list<pair<int, int>> paths[10];
// 当前路径的迭代器
list<pair<int, int>>::iterator current_index[10];

struct Robot
{
    int x, y, goods;
    int status;
    int mbx, mby;
    int zt;
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

deque<Item> items_exist;
multiset<Item> items_set;

void Init()
{
    for(int i = 1; i <= n; i ++)
        scanf("%s", ch[i]+1);
    
    // 读入泊位信息
    for(int i = 0; i < berth_num; i ++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        berth[id].x ++;
        berth[id].y ++;
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}


int Input()
{
    // 当前帧和金钱
    scanf("%d%d", &id, &money);
    int num;
    // 读入新增物品信息
    scanf("%d", &num);
    for(int i = 1; i <= num; i ++)
    {
        // 新增物品的位置和价值
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        gds[x+1][y+1] = val;
        Item item = Item(x+1, y+1, val, id);
        items_exist.push_back(item);
        items_set.insert(item);
    }
    while(!items_exist.empty() && items_exist.front().appear_frame <= id - 1000) {
        Item item_del = items_exist.front();
        items_set.erase(items_set.find(item_del));
        gds[item_del.x][item_del.y] = 0;
        items_exist.pop_front();
    }

    // 读入机器人信息
    for(int i = 0; i < robot_num; i ++)
    {
        int sts;
        // 是否携带货物，坐标，状态
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &sts);
        robot[i].x ++;
        robot[i].y ++;
    }

    // 读入船只信息
    for(int i = 0; i < boat_num; i ++)
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);
    char okk[100];
    scanf("%s", okk);
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

// A*搜索算法
bool AStarSearch(Item target, int robot_index) {
    priority_queue<Node> open_list;
    bool closed_list[N][N] = {false};
    // 用来获得路径
    Node parents[N][N];
    int start_x = robot[robot_index].x, start_y = robot[robot_index].y;
    open_list.push({start_x, start_y, 0, Heuristic(start_x, start_y, target.x, target.y), {-1, -1}});

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (current.x == target.x && current.y == target.y) {
            // 保存路径
            for (Node p = current; p.x != start_x || p.y != start_y; p = parents[p.x][p.y]) {
                paths[robot_index].push_front({p.x, p.y});
            }
            paths[robot_index].push_front({start_x, start_y});
            return true;
        }

        if (closed_list[current.x][current.y]) {
            continue;  // 已经检查过这个节点
        }
        closed_list[current.x][current.y] = true;

        // 检查周围的节点
        for (int i = 0; i < 4; i++) {
            int nx = current.x + dxs[i];
            int ny = current.y + dys[i];
            if (nx >= 1 && nx < 201 && ny >= 1 && ny < 201 && !closed_list[nx][ny] && ch[nx][ny] != '*' && ch[nx][ny] != '#') {
                parents[nx][ny] = current;
                open_list.push({nx, ny, current.g + 1, Heuristic(nx, ny, target.x, target.y), {current.x, current.y}});
            }
        }
    }
    return false;
}

int get_direction(pair<int, int> a, pair<int, int> b) {
    // a:当前坐标 b:下一时刻
    if(a.first == b.first) {
        if(a.second < b.second) {
            return 0;
        } else {
            return 1;
        }
    } else {
        if(a.first < b.first) {
            return 3;
        } else {
            return 2;
        }
    }
}

int main()
{   
    f1.open("data.txt",ios::out);
    Init();
    int index = 0;
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
        int id = Input();
        // 机器人部分
        // 因为时间限制，每一轮能A*的机器人数量需要限制
        int astar_time = 0;
        for(int bot_num = 0; bot_num < robot_num; bot_num++){
            // 机器人包括五个状态
            // 0:寻找去货物的最短路 1:运输途中 2: 到达货物点 3: 寻找到泊位的最短路 4: 到达泊位
            switch (robot[bot_num].zt)
            {
            // 所有机器人的初始状态，没有物品，寻路去找物品
            case 0:{
                if(astar_time > 1) break;
                astar_time ++;
                // 遍历所有货物
                int count = 0;
                for(auto it = items_set.begin(); it != items_set.end(); it ++) {
                    if(count > 5) break;
                    if(robot[bot_num].goods == 0) {
                        if(AStarSearch(*it, bot_num)) {
                            // 如果寻到路径，这里不break switch，在当前帧进入状态1
                            robot[bot_num].zt = 1;
                            current_index[bot_num] = paths[bot_num].begin();
                            items_set.erase(it);
                            break;
                        }
                    }
                    count ++;
                }
                // 没有找到最短路，跳过此机器人
                // 如果不为0，说明找到了最短路，可以走一步
                if(robot[bot_num].zt == 0) {
                    break;
                }
            }
            case 1:{
                // 出现了丢帧
                if(robot[bot_num].x != current_index[bot_num] -> first && robot[bot_num].y != current_index[bot_num] -> second) {
                    f1 << "re find!" << endl;
                    paths[bot_num].clear();
                    current_index[bot_num] = paths[bot_num].begin();
                    robot[bot_num].zt = 0;
                    break;
                }
                // 系统确认到达目的地
                if(robot[bot_num].x == paths[bot_num].back().first && robot[bot_num].y == paths[bot_num].back().second){
                    // TODO:可以进行小判断，如果已经机器人已经拾取，就进入状态3，没拾取重新拾取，提前一帧
                    robot[bot_num].zt = 2;
                    break;
                }
                current_index[bot_num] ++;
                int direction = get_direction({robot[bot_num].x, robot[bot_num].y}, *current_index[bot_num]);
                f1 << "move " << bot_num << " " << direction << endl;
                printf("move %d %d\n", bot_num, direction);
                // 提前拾取
                if(current_index[bot_num] == paths[bot_num].end()) {
                    printf("get %d\n", bot_num);
                }
                break;

            }
            // 能进入状态2，就是已经站在物品点了
            case 2:{
                // 被提前拾取
                if(robot[bot_num].goods == 1) {
                    robot[bot_num].zt = 3;
                }
                else{
                    if(gds[robot[bot_num].x][robot[bot_num].y] != 0) {
                        printf("get %d\n", bot_num);
                    }
                    // 物品消失，重新寻路
                    else{
                        robot[bot_num].zt = 0;
                    }
                }
                break;
            }
            // 去泊位，能进入状态3，就是已经有物品了
            case 3:{
                
            }
            // 进入状态4，说明已经到达泊位
            case 4:{

            }
            default:
                break;
            }
            
        }

        
        puts("OK");
        fflush(stdout);
    }
    f1.close();
    return 0;
}
