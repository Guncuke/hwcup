#include <bits/stdc++.h>
#include<fstream>
using namespace std;
fstream f1;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;
const int dxs[] = {0, 0, -1, 1};
const int dys[] = {1, -1, 0, 0};
// 金钱，船只容量，当前帧数
int money, boat_capacity, id;
// 地图
char ch[N][N];
// 物品位置
int gds[N][N];

struct Robot
{
    int x, y, goods;
    int status;
    int mbx, mby;
    int zt=0;
    int target_berth;
    // 当前机器人的路径和当前位置
    list<pair<int, int>> path;
    list<pair<int, int>>::iterator current_index;
    list<pair<int, int>>::iterator next_index;
    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
    void clearPath(){
        path.clear();
        current_index = path.end();
        next_index = path.end();
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
    // 泊位物品
    int items_num=0;
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
    // arrive_time: 到达时间
    int num=0, pos, status, arrive_time;
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
        f1 << "erase" << items_exist.front().appear_frame << endl;
        items_set.erase(items_exist.front());
        gds[items_exist.front().x][items_exist.front().y] = 0;
        items_exist.pop_front();
    }

    // 读入机器人信息
    for(int i = 0; i < robot_num; i ++)
    {
        int sts;
        // 是否携带货物，坐标，状态
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &robot[i].status);
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
int HeuristicItem(int x, int y, int xx, int yy)
{
    return abs(x - xx) + abs(y - yy);
}

// A*寻找物品
bool AStarSearchItem(Item target, Robot& bot) {
    priority_queue<Node> open_list;
    bool closed_list[N][N] = {false};
    // 用来获得路径
    Node parents[N][N];
    int start_x = bot.x, start_y = bot.y;
    open_list.push({start_x, start_y, 0, HeuristicItem(start_x, start_y, target.x, target.y), {-1, -1}});

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (current.x == target.x && current.y == target.y) {
            // 保存路径
            for (Node p = current; p.x != start_x || p.y != start_y; p = parents[p.x][p.y]) {
                bot.path.push_front({p.x, p.y});
            }
            bot.path.push_front({start_x, start_y});
            bot.current_index = bot.path.begin();
            bot.next_index = bot.path.begin();
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
                open_list.push({nx, ny, current.g + 1, HeuristicItem(nx, ny, target.x, target.y), {current.x, current.y}});
            }
        }
    }
    return false;
}

int HeuristicBerth(int x, int y, int xx, int yy) {
    int center_x = yy + 2;
    int center_y = xx + 2;
    return abs(x - center_x) + abs(y - center_y);
}

// A*寻找泊位
bool AStarSearchBerth(Berth& target, Robot& bot) {
    priority_queue<Node> open_list;
    bool closed_list[N][N] = {false};
    // 用来获得路径
    Node parents[N][N];
    int start_x = bot.x, start_y = bot.y;
    open_list.push({start_x, start_y, 0, HeuristicBerth(start_x, start_y, target.x, target.y), {-1, -1}});

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (current.x >= target.x && current.x < target.x + 4 && current.y >= target.y && current.y < target.y + 4) {
            // 保存路径
            for (Node p = current; p.x != start_x || p.y != start_y; p = parents[p.x][p.y]) {
                bot.path.push_front({p.x, p.y});
            }
            bot.path.push_front({start_x, start_y});
            bot.current_index = bot.path.begin();
            bot.next_index = bot.path.begin();
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
                open_list.push({nx, ny, current.g + 1, HeuristicBerth(nx, ny, target.x, target.y), {current.x, current.y}});
            }
        }
    }
    return false;
}

int GetDirection(pair<int, int> a, pair<int, int> b) {
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
        f1 << "id: " << id << endl;
        // 机器人部分
        // 因为时间限制，每一轮能A*的机器人数量需要限制
        int astar_time = 0;
        for(int bot_num = 0; bot_num < robot_num; bot_num++){
            Robot& bot = robot[bot_num];
            // 防止意外，不知道什么时候机器人进入恢复状态
            if(bot.status==0){
                bot.zt=6;
            }
            // 机器人包括6个状态
            // 0:寻找去货物的最短路 1:运输途中 2: 到达货物点 3: 寻找到泊位的最短路 4：去往泊位 5: 到达泊位 6：恢复状态
            switch (bot.zt)
            {
            // 所有机器人的初始状态，没有物品，寻路去找物品
            case 0:{
                f1 << bot_num << ' ' << 0 << endl;
                if(astar_time > 3) break;
                astar_time ++;
                int count = 0;
                bot.clearPath();
                // TODO: 估算物品距离和价值，综合排序（优先队列）
                // 距离不是机器人到物品的距离，是物品到泊位的min距离
                for(auto it = items_set.begin(); it != items_set.end(); it ++) {
                    if(count > 5) break;
                    if(AStarSearchItem(*it, bot)) {
                        bot.zt = 1;
                        items_set.erase(it);
                        break;
                    }
                    count ++;
                }
                // 没有找到最短路，跳过此机器人
                if(bot.zt == 0) {
                    break;
                }
                // 正常找到路径，此帧可以继续走一步，不break
            }
            // 去物品点状态
            case 1:{
                f1 << bot_num << ' '<< 1 << endl;
                // 出现了与预测不符的情况，重新寻路
                if(bot.x != bot.next_index -> first && bot.y != bot.next_index -> second) {
                    f1 << "re find!" << endl;
                    bot.zt = 0;
                    break;
                }
                else{
                    bot.current_index = bot.next_index;
                }
                // 系统确认到达目的地
                if(bot.x == bot.path.back().first && bot.y == bot.path.back().second){
                    // 可以进行小判断，如果已经机器人已经拾取，就进入状态3，没拾取重新拾取，提前一帧
                    if(bot.goods == 1) {
                        bot.zt = 3;
                    }
                    else{
                        printf("get %d\n", bot_num);
                        bot.zt = 2;
                    }
                    break;
                }
                bot.next_index = next(bot.current_index);
                int direction = GetDirection(*bot.current_index, *bot.next_index);
                printf("move %d %d\n", bot_num, direction);
                // 提前拾取
                if(bot.next_index == bot.path.end()) {
                    printf("get %d\n", bot_num);
                }
                break;
            }
            // 能进入状态2，就是已经站在物品点了
            case 2:{
                f1 << bot_num << ' ' << 2 << endl;
                // 被提前拾取
                if(bot.goods == 1) {
                    bot.zt = 3;
                }
                else{
                    if(gds[bot.x][bot.y] != 0) {
                        printf("get %d\n", bot_num);
                    }
                    // 物品消失，重新寻路
                    else{
                        bot.zt = 0;
                    }
                }
                break;
            }
            // 寻找泊位，能进入状态3，就是已经有物品了
            case 3:{
                f1 << bot_num << ' ' << 3 << endl;
                if(astar_time > 3) break;
                astar_time ++;
                int count = 0;
                bot.clearPath();
                // TODO: 估算泊位距离和价值，综合排序（优先队列）
                for(int i = 0; i < berth_num; i ++){
                    if(count > 5) break;
                    if(AStarSearchBerth(berth[bot_num/2], bot)) {
                        bot.zt = 4;
                        bot.target_berth = bot_num/2;
                        break;
                    }
                    count ++;
                }
                // 没有找到最短路，跳过此机器人
                if(bot.zt == 3) {
                    break;
                }
                // 寻到路径直接进入状态4
            }
            // 去泊位状态
            case 4:{
                f1 << bot_num << ' ' << 4 << endl;
                // 出现了与预测不符的情况，重新寻路
                if(bot.x != bot.next_index -> first && bot.y != bot.next_index -> second) {
                    f1 << "re find!" << endl;
                    bot.zt = 3;
                    break;
                }
                else{
                    bot.current_index = bot.next_index;
                }
                // 系统确认到达目的地
                if(bot.x == bot.path.back().first && bot.y == bot.path.back().second){
                    // 可以进行小判断，如果已经机器人已经放下，就进入状态0，没拾取重新拾取，提前一帧
                    if(bot.goods == 0) {
                        bot.zt = 0;
                        berth[bot.target_berth].items_num++;
                    }
                    else{
                        printf("pull %d\n", bot_num);
                        bot.zt = 5;
                    }
                    break;
                }
                bot.next_index = next(bot.current_index);
                int direction = GetDirection(*bot.current_index, *bot.next_index);
                printf("move %d %d\n", bot_num, direction);
                // 提前放下
                if(bot.next_index == bot.path.end()) {
                    printf("pull %d\n", bot_num);
                }
                break;
            }
            case 5:{
                f1 << bot_num << ' ' << 5 << endl;
                // 被放下
                if(bot.goods == 0) {
                    bot.zt = 0;
                    berth[bot.target_berth].items_num++;
                }
                else{
                    printf("pull %d\n", bot_num);
                }
                break;
            }
            case 6:{
                f1 << bot_num << ' ' << 6 << endl;
                // 机器人恢复了
                if(bot.status==1){
                    if(bot.goods == 1){
                        bot.zt = 3;
                    }
                    else{
                        bot.zt = 0;
                    }
                }
                break;
            }
            default:
                break;
            }
            
        }

        for(int i = 0; i < boat_num; i ++)
        {   
            // 如果在虚拟点，出发
            if(boat[i].status == 1 && boat[i].pos== -1) {
                boat[i].num = 0;
                printf("ship %d %d\n", i, i);
            }
            // 到达泊位
            else if(boat[i].status == 1 && boat[i].pos != -1) {
                // 目标泊位
                int target_berth = boat[i].pos;
                // 泊位没有货物，此帧等待
                if(berth[target_berth].items_num == 0){
                    break;
                }
                // 有货物，装载
                else{
                    // 泊位可以给那么多
                    int upload_num = min(berth[target_berth].loading_speed, berth[target_berth].items_num);
                    // 船上还能装多少
                    upload_num = min(upload_num, boat_capacity - boat[i].num);
                    boat[i].num += upload_num;
                    berth[target_berth].items_num -= upload_num;
                    // 没货了就走
                    if(berth[target_berth].items_num == 0) {
                        printf("go %d\n", i);
                    }
                }
            }
        }
        puts("OK");
        fflush(stdout);
    }
    f1.close();
    return 0;
}
