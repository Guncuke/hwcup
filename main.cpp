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
// 一个机器人如果失败max_astar_fail次，就不再寻路
const int max_astar_fail = 20;
// 每一帧可以做的A*次数
const int max_astar_time = 2;
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
    int zt;
    // 机器人目标泊位
    int target_berth;
    // 机器人身上物品的价值
    int value;
    // 当前机器人的路径和当前位置
    vector<pair<int, int>> path;
    int current_index;
    // A星失败次数
    int astar_fail;
    Robot() {}
    Robot(int startX, int startY) {
        x = startX;
        y = startY;
    }
    void clearPath(){
        path.clear();
        current_index = -1;
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
    // 泊位物品价值
    int value;
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
    int num=0, pos, status;
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

// 冲突解决方案
// A*拓展节点对应步与已有路径不冲突
bool Collision(int x1, int y1, int x2, int y2, int g, int bot_id){
    // 其他机器人在第g步的预测值
    int predict_x1, predict_y1, predict_x2, predict_y2;
    for(int i = 0; i < robot_num; i++){
        if(i == bot_id) continue;
        if(robot[i].zt == 1 || robot[i].zt == 4){
            if(robot[i].current_index+g+1 < robot[i].path.size()){
                predict_x1 = robot[i].path[robot[i].current_index+g].first;
                predict_y1 = robot[i].path[robot[i].current_index+g].second;
                predict_x2 = robot[i].path[robot[i].current_index+g+1].first;
                predict_y2 = robot[i].path[robot[i].current_index+g+1].second;
                // 直接撞上
                if(x2 == predict_x2 && y2 == predict_y2){
                    return false;
                }
                // 左右对撞
                if(x1 == x2 && predict_x1 == predict_x2 && predict_x1 == x1 && y2 == predict_y1 && y1 == predict_y2){
                    return false;
                }
                // 上下对撞
                if(y1 == y2 && predict_y1 == predict_y2 && predict_y1 == y1 && x2 == predict_x1 && x1 == predict_x2){
                    return false;
                }
            }
        }
        else{
            if(x2 == robot[i].x && y2 == robot[i].y || x1 == robot[i].x && y1 == robot[i].y)
                return false;
        }
    }
    return true;
}
// A*寻找物品
bool AStarSearchItem(Item target, Robot& bot, int bot_id) {
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
                bot.path.push_back({p.x, p.y});
            }
            bot.path.push_back({start_x, start_y});
            reverse(bot.path.begin(), bot.path.end());
            bot.current_index = 0;
            bot.mbx = target.x;
            bot.mby = target.y;
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
            // current.g刚好是第一步的帧数，就是当前的位置
            // 那么current.g就可以代表当前这个位置是第几步
            // 我们就可以通过current.g来同样取得目前已知其他机器人在current.g时刻的位置
            // 处于1，4状态的机器人，需要偏移current.g帧获取位置
            // 处于其他状态的机器人保持在当前位置
            // 需要判断的冲突有2个：一个是机器人走到同一个位置，另一个是机器人相互走到对方的位置
            if (nx >= 1 && nx < 201 && ny >= 1 && ny < 201 && !closed_list[nx][ny] && ch[nx][ny] != '*' && ch[nx][ny] != '#' && current.g <= 1000 && Collision(current.x, current.y, nx, ny, current.g, bot_id)) {
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
bool AStarSearchBerth(Berth& target, Robot& bot, int bot_id) {
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
                bot.path.push_back({p.x, p.y});
            }
            bot.path.push_back({start_x, start_y});
            reverse(bot.path.begin(), bot.path.end());
            bot.current_index = 0;
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
            if (nx >= 1 && nx < 201 && ny >= 1 && ny < 201 && !closed_list[nx][ny] && ch[nx][ny] != '*' && ch[nx][ny] != '#'
            && Collision(current.x, current.y, nx, ny, current.g, bot_id)) {
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
        // 首先要判断服务器是否正确移动，如果没有正确移动，回到寻路状态
        // 然后判断机器人是否处于正常状态，如果处于恢复状态去状态6
        for(int i = 0; i < robot_num; i ++)
        {
            Robot& bot = robot[i];
            // 如果是恢复状态，进入状态6
            if(bot.status==0){
                bot.zt=6;
                bot.clearPath();
                continue;
            }
            if(bot.zt == 1 || bot.zt == 4) {
                // 机器人没有移动，重新寻路
                if(bot.x != bot.path[bot.current_index+1].first || bot.y != bot.path[bot.current_index+1].second) {
                    f1 << "before re find!" << endl;
                    bot.zt--;
                    continue;
                }
                // 正常移动
                robot[i].current_index++;
            }
        }
        // 之后，所有处于寻路状态的机器人位置都正确，位于current_index上
        int astar_time = 0;
        for(int bot_num = 0; bot_num < robot_num; bot_num++){
            Robot& bot = robot[bot_num];
            // 机器人包括6个状态
            // 0:寻找去货物的最短路 1:运输途中 2: 到达货物点 3: 寻找到泊位的最短路 4：去往泊位 5: 到达泊位 6：恢复状态
            switch (bot.zt)
            {
            // 状态0：寻路物品
            case 0:{
                find_item:
                f1 << bot_num << " " << 0 << endl;
                bot.clearPath();
                if(bot.astar_fail >= max_astar_fail) break;
                if(astar_time >= max_astar_time) break;
                astar_time ++;
                int count = 0;
                // TODO: 估算物品距离和价值，综合排序（优先队列）
                // 绝对值距离如果超出剩余帧数，就不用寻路了
                // 根据观察可以发现，场上同时出现的物品不会太多，在100个以内
                for(auto it = items_set.begin(); it != items_set.end(); it ++) {
                    if(count > 5) break;
                    if(AStarSearchItem(*it, bot, bot_num)) {
                        bot.zt = 1;
                        items_set.erase(it);
                        break;
                    }
                    // A星失败次数
                    bot.astar_fail++;
                    count ++;
                }
                // 没有找到最短路，跳过此机器人
                if(bot.zt == 0) {
                    break;
                }
                // 正常找到路径，此帧可以继续走一步，不break
            }
            // 状态2：去物品点途中
            case 1:{
                f1 << bot_num << " " << 1 << endl;
                // 系统确认到达目的地
                if(bot.x == bot.path.back().first && bot.y == bot.path.back().second){
                    bot.zt = 2;
                }
                // 还在路上
                else{
                    // 物品消失
                    if(gds[bot.mbx][bot.mby] == 0) {
                        bot.zt = 2;
                        goto find_item;
                    }
                    int direction = GetDirection(bot.path[bot.current_index], bot.path[bot.current_index+1]);
                    printf("move %d %d\n", bot_num, direction);
                    // 提前拾取
                    if(bot.current_index+1 == bot.path.size() - 1){
                        printf("get %d\n", bot_num);
                    }
                    break;
                }
            }
            // 状态2：到达物品点
            case 2:{
                f1 << bot_num << " " << 2 << endl;
                // 被拾取
                if(bot.goods == 1) {
                    bot.value = gds[bot.mbx][bot.mby];
                    gds[bot.mbx][bot.mby] = 0;
                    bot.zt = 3;
                }
                else{
                    if(gds[bot.x][bot.y] != 0) {
                        printf("get %d\n", bot_num);
                    }
                    // 物品消失，重新寻路
                    else{
                        bot.zt = 0;
                        bot.value = 0;
                    }
                    break;
                }
            }
            // 状态3：寻找泊位
            case 3:{
                find_berth:
                f1 << bot_num << " " << 3 << endl;
                bot.clearPath();
                if(astar_time >= max_astar_time) break;
                astar_time ++;
                int count = 0;
                // TODO: 估算泊位距离和价值，综合计算选出最优可达的泊位（优先队列？）
                for(int i = 0; i < berth_num; i ++){
                    if(count > 5) break;
                    if(AStarSearchBerth(berth[bot_num/2], bot, bot_num)) {
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
            // 状态4：去泊位途中
            case 4:{
                f1 << bot_num << " " << 4 << endl;
                // 系统确认到达目的地
                if(bot.x == bot.path.back().first && bot.y == bot.path.back().second){
                    bot.zt = 5;
                }
                // 在路上
                else{
                    int direction = GetDirection(bot.path[bot.current_index], bot.path[bot.current_index+1]);
                    printf("move %d %d\n", bot_num, direction);
                    // 提前放下
                    if(bot.current_index+1 == bot.path.size() - 1){
                        printf("pull %d\n", bot_num);
                    }
                    break;
                }
            }
            // 状态5：到达泊位
            case 5:{
                f1 << bot_num << " " << 5 << endl;
                // 被放下
                if(bot.goods == 0) {
                    bot.zt = 0;
                    bot.value = 0;
                    berth[bot.target_berth].items_num++;
                    berth[bot.target_berth].value += bot.value;
                    goto find_item;
                }
                else{
                    printf("pull %d\n", bot_num);
                    break;
                }
            }
            // 状态6：恢复状态
            case 6:{
                f1 << bot_num << " " << 6 << endl;
                // 机器人恢复正常
                if(bot.status==1){
                    if(bot.goods == 1){
                        bot.zt = 3;
                        goto find_berth;
                    }
                    else{
                        bot.zt = 0;
                        goto find_item;
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
