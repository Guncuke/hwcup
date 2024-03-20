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
// 物品平衡二叉树排序的权重
const float quanzhong_distance = 50;
const float quanzhong_value = 0.1;
// 泊位效率，金钱，和时间的权重
const float quanzhong_efficiency = 100;
const float quanzhong_money = 1;
const float quanzhong_time = 2;
// 金钱，船只容量，当前帧数
int money, boat_capacity, id;
// 泊位中心点
int berth_center[10][2];
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
    int current_index=-1;
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
    int id;
    // 泊位左上角位置，泊位大小4*4
    int x;
    int y;
    // 泊位到虚拟点需要的时间
    int transport_time;
    // 泊位装载速度
    int loading_speed;
    // 泊位上的物品价值链表
    deque<int> items;
    // 总价值
    int total_value;
    // 是否被占用(有船在装卸和有船前往都算被占用)
    bool is_occupied;
    Berth(){}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this -> x = x;
        this -> y = y;
        this -> transport_time = transport_time;
        this -> loading_speed = loading_speed;
    }
}berth[berth_num + 10];

// 泊位优先级
vector<Berth> berth_priority(berth_num);

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

    bool operator<(const Item& rhs) const {
        return val  > rhs.val;
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
        berth[id].id = id;
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        berth[id].x ++;
        berth[id].y ++;
    }
    scanf("%d", &boat_capacity);
    char okk[100];
    scanf("%s", okk);
    // 泊位中心
    for(int i =0; i < berth_num; i++){
        berth_center[i][0] = berth[i].x + 2;
        berth_center[i][1] = berth[i].y + 2;
    }
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
bool Collision(int x1, int y1, int x2, int y2, int step, int bot_id){
    // 其他机器人在第g步的预测值
    int predict_x1, predict_y1, predict_x2, predict_y2;
    for(int i = 0; i < robot_num; i++){
        if(i == bot_id) continue;
        if(robot[i].zt == 1 || robot[i].zt == 4){
            if(robot[i].current_index+step+1 < robot[i].path.size()){
                predict_x1 = robot[i].path[robot[i].current_index+step].first;
                predict_y1 = robot[i].path[robot[i].current_index+step].second;
                predict_x2 = robot[i].path[robot[i].current_index+step+1].first;
                predict_y2 = robot[i].path[robot[i].current_index+step+1].second;
                // 直接撞上
                if(x2 == predict_x2 && y2 == predict_y2){
                    return false;
                }
                // 左右对撞
                if(x1 == predict_x1 && x2 == predict_x2 && y2 == predict_y1 && y1 == predict_y2){
                    return false;
                }
                // 上下对撞
                if(y1 == predict_y1 && y2 == predict_y2 && x2 == predict_x1 && x1 == predict_x2){
                    return false;
                }
            }
        }
        else{
            if(x2 == robot[i].x && y2 == robot[i].y)
                return false;
        }
    }
    return true;
}
// A*寻找物品
bool AStarSearchItem(Item target, Robot& bot, int bot_id) {
    priority_queue<Node> open_list;
    bool closed_list[N][N] = {false};
    int open_list_dj[N][N];
    fill(*open_list_dj, *open_list_dj + N * N, -1);
    // 用来获得路径
    Node parents[N][N];
    int start_x = bot.x, start_y = bot.y;
    open_list.push({start_x, start_y, 0, HeuristicItem(start_x, start_y, target.x, target.y)});

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
        // 节点加入close_list
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
            if (nx >= 1 && nx < 201 && ny >= 1 && ny < 201 && !closed_list[nx][ny] && ch[nx][ny] != '*' && ch[nx][ny] != '#' && current.g <= 1000 
            && Collision(current.x, current.y, nx, ny, current.g, bot_id)) {
                int new_g = current.g + 1;
                int new_h = HeuristicItem(nx, ny, target.x, target.y);
                // 在Openlist里，即被发现过，但是未被拓展
                if(open_list_dj[nx][ny] != -1){
                    if(!closed_list[nx][ny] && new_g+new_h < open_list_dj[nx][ny]){
                        open_list_dj[nx][ny] = new_g+new_h;
                        open_list.push({nx, ny, new_g, new_h});
                        parents[nx][ny] = current;
                    }
                }
                // open_list_dj还是-1，未被发现过
                else{
                    open_list.push({nx, ny, new_g, new_h});
                    parents[nx][ny] = current;
                    open_list_dj[nx][ny] = new_g+new_h;
                }
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
    int open_list_dj[N][N];
    fill(*open_list_dj, *open_list_dj + N * N, -1);
    // 用来获得路径
    Node parents[N][N];
    int start_x = bot.x, start_y = bot.y;
    open_list.push({start_x, start_y, 0, HeuristicBerth(start_x, start_y, target.x, target.y)});

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
                int new_g = current.g + 1;
                int new_h = HeuristicItem(nx, ny, target.x, target.y);
                // 在Openlist里，即被发现过，但是未被拓展
                if(open_list_dj[nx][ny] != -1){
                    if(!closed_list[nx][ny] && new_g+new_h < open_list_dj[nx][ny]){
                        open_list_dj[nx][ny] = new_g+new_h;
                        open_list.push({nx, ny, new_g, new_h});
                        parents[nx][ny] = current;
                    }
                }
                // open_list_dj还是-1，未被发现过
                else{
                    open_list.push({nx, ny, new_g, new_h});
                    parents[nx][ny] = current;
                    open_list_dj[nx][ny] = new_g+new_h;
                }
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
                    bot.zt--;
                    continue;
                }
                // 正常移动
                robot[i].current_index++;
            }
        }
        // 之后，所有处于寻路状态的机器人位置都正确，位于current_index上
        for(int bot_num = 0; bot_num < robot_num; bot_num++){
            Robot& bot = robot[bot_num];
            // 机器人包括6个状态
            // 0:寻找去货物的最短路 1:运输途中 2: 到达货物点 3: 寻找到泊位的最短路 4：去往泊位 5: 到达泊位 6：恢复状态
            switch (bot.zt)
            {
            // 状态0：寻路物品
            case 0:{
                find_item:
                bot.clearPath();
                // items_set是一个平衡二叉树，不会超过100个物品，直接遍历，获得物品的位置，价格和剩余帧数
                // TODO: 调节价值和距离的权重
                vector<Item> items(items_set.begin(), items_set.end());

                // 排序
                sort(items.begin(), items.end(), [&](const Item& a, const Item& b) {
                    return -(abs(bot.x-a.x) + abs(bot.y-a.y)) * quanzhong_distance + a.val * quanzhong_value > -(abs(bot.x-b.x) + abs(bot.y-b.y)) * quanzhong_distance + b.val * quanzhong_value;
                });
                for(auto it = items.begin(); it != items.end(); it ++) {
                    // 如果剩余帧数比最短路还要少，跳过 预留100帧
                    if((abs(bot.x-it->x) + abs(bot.y-it->y)) > (900 - (id - it->appear_frame))) continue;
                    if(AStarSearchItem(*it, bot, bot_num)) {
                        bot.zt = 1;
                        items_set.erase(*it);
                        break;
                    }
                }
                // 没有找到最短路，跳过此机器人
                if(bot.zt == 0) {
                    break;
                }
                // 正常找到路径，此帧可以继续走一步，不break
            }
            // 状态2：去物品点途中
            case 1:{
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
                bot.clearPath();
                // 估算泊位距离和价值，综合计算选出最优可达的泊位
                // TODO: 现在直接按距离排序，后续可能得加上泊位装载速度
                vector<pair<int, int>> berth_distance(10);
                for(int i = 0; i < berth_num; i ++){
                    berth_distance[i].first = abs(bot.x - berth_center[i][0]) + abs(bot.y - berth_center[i][1]);
                    berth_distance[i].second = i;
                }
                sort(berth_distance.begin(), berth_distance.end());
                for(int i = 0; i < berth_num; i ++){
                    if(AStarSearchBerth(berth[berth_distance[i].second], bot, bot_num)) {
                        bot.zt = 4;
                        bot.target_berth = berth_distance[i].second;
                        break;
                    }
                }
                // 没有找到最短路，跳过此机器人
                if(bot.zt == 3) {
                    break;
                }
                // 寻到路径直接进入状态4
            }
            // 状态4：去泊位途中
            case 4:{
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
                // 被放下
                if(bot.goods == 0) {
                    bot.zt = 0;
                    berth[bot.target_berth].items.push_back(bot.value);
                    berth[bot.target_berth].total_value += bot.value;
                    bot.value = 0;
                    goto find_item;
                }
                else{
                    printf("pull %d\n", bot_num);
                    break;
                }
            }
            // 状态6：恢复状态
            case 6:{
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

        // 0: 移动 1: 正常 2:等待
        // 出发选择
        // 将berth_priority复制一份，用于排序
        for(int i = 0; i < berth_num; i++){
            berth_priority[i] = berth[i];
        }
        // 根据泊位价值/效率/时间排序
        sort(berth_priority.begin(), berth_priority.end(), [&](const Berth& a, const Berth& b) {
            return a.total_value * quanzhong_money + a.loading_speed * quanzhong_efficiency - a.transport_time * quanzhong_time > b.total_value * quanzhong_money + b.loading_speed * quanzhong_efficiency - b.transport_time * quanzhong_time;
        });

        for(int i = 0; i < boat_num; i++){
            // TODO: 如果在虚拟点，通过1.泊位价值+2.路径花费时间+3.装载速度 决定去哪个泊位
            if(boat[i].status == 1 && boat[i].pos== -1) {
                boat[i].num = 0;
                for(int j = 0; j < berth_num; j++){
                    // 被占用的泊位不考虑
                    if(berth[berth_priority[j].id].is_occupied) continue;
                    berth[berth_priority[j].id].is_occupied = true;
                    printf("ship %d %d\n", i, berth_priority[j].id);
                    break;
                }
            }
        }

        // 泊位装货，维持泊位信息与出发
        for(int i = 0; i < boat_num; i ++)
        {   
            // 在泊位
            if(boat[i].status == 1 && boat[i].pos != -1) {
                berth[boat[i].pos].is_occupied = true;
                int target_berth = boat[i].pos;
                // 泊位没有货物，等待
                if(berth[target_berth].items.size() == 0){
                    break;
                }
                // 有货物，装载
                else{
                    // 泊位可以给那么多
                    int upload_num = min(berth[target_berth].loading_speed, (int)berth[target_berth].items.size());
                    // 船上还能装多少
                    upload_num = min(upload_num, boat_capacity - boat[i].num);
                    boat[i].num += upload_num;
                    for(int j = 0; j < upload_num; j ++){
                        berth[target_berth].total_value -= berth[target_berth].items.front();
                        berth[target_berth].items.pop_front();
                    }                    
                    // 货满了出发
                    if(boat[i].num == boat_capacity) {
                        berth[boat[i].pos].is_occupied = false;
                        printf("go %d\n", i);
                        break;
                    }
                    // 没货了，直接出发
                    if(berth[target_berth].items.size() == 0) {
                        berth[boat[i].pos].is_occupied = false;
                        printf("go %d\n", i);
                        break;
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
