#include <bits/stdc++.h>
#include<fstream>
using namespace std;
// fstream f1;

const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 210;
const int dxs[] = {0, 0, -1, 1};
const int dys[] = {1, -1, 0, 0};
// 泊位效率，金钱，和时间的权重
const float quanzhong_efficiency = 10;
const float quanzhong_money = 1;
const float quanzhong_time = 1;
// 金钱，船只容量，当前帧数
int money, boat_capacity, id;
// 地图
char ch[N][N];
// 物品位置
int gds[N][N];
// 物品出现帧数
int gds_frame[N][N];


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
    // 机器人寻路失败次数
    int bfs_fail_time;
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

deque<Item> items_exist;

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
        gds_frame[x+1][y+1] = id;
        Item item = Item(x+1, y+1, val, id);
        items_exist.push_back(item);
    }
    while(!items_exist.empty() && items_exist.front().appear_frame <= id - 1000) {
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

// 冲突解决方案
// 拓展节点对应步与已有路径不冲突
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
            else{
                predict_x2 = robot[i].path.back().first;
                predict_y2 = robot[i].path.back().second;
                if(x2 == predict_x2 && y2 == predict_y2){
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

// BFS寻物
vector<vector<pair<int, int>>> BFSItem(pair<int, int> start, int bot_id) {
    int N = 201;
    vector<vector<bool>> visited(N, vector<bool>(N, false));
    // 存储坐标和路径
    queue<pair<pair<int, int>, vector<pair<int, int>>>> q; 
    // 存储每个目标点的路径
    vector<vector<pair<int, int>>> paths; 

    q.push({start, {start}}); // 起始坐标和路径
    visited[start.first][start.second] = true;

    while (!q.empty()) {
        pair<int, int> pos = q.front().first;
        vector<pair<int, int>> path = q.front().second;
        int length = path.size()-1;
        // 长度超过一定程度不再搜索
        if(length >= 500) continue;
        q.pop();
        // 如果当前位置有物品，记录路径
        if (gds[pos.first][pos.second] != 0) {
            paths.push_back(path);
        }
        // 遍历当前位置的四个方向
        for (int i = 0; i < 4; i++) {
            int new_x = pos.first + dxs[i];
            int new_y = pos.second + dys[i];
            // 检查新坐标是否在矩阵范围内且是空地且未访问过
            if (new_x >= 1 && new_x < N && new_y >= 1 && new_y < N && !visited[new_x][new_y] && ch[new_x][new_y] != '*' && ch[new_x][new_y] != '#' && Collision(pos.first, pos.second, new_x, new_y, length, bot_id)) {
                vector<pair<int, int>> new_path = path;
                new_path.push_back({new_x, new_y});
                q.push({{new_x, new_y}, new_path});
                visited[new_x][new_y] = true;
            }
        }
    }
    return paths;
}


// BFS寻泊
pair<int, vector<pair<int, int>>> BFSBerth(pair<int, int> start, int bot_id) {
    int N = 201;
    vector<vector<bool>> visited(N, vector<bool>(N, false));
    // 存储坐标和路径
    queue<pair<pair<int, int>, vector<pair<int, int>>>> q; 

    q.push({start, {start}}); // 起始坐标和路径
    visited[start.first][start.second] = true;

    while (!q.empty()) {
        pair<int, int> pos = q.front().first;
        vector<pair<int, int>> path = q.front().second;
        int length = path.size()-1;
        q.pop();
        // 如果到达泊位，记录路径
        // 泊位的大小为4*4，左上角为berth[i].x 和berth[i].y
        for(int i = 0; i < berth_num; i++){
            if(pos.first >= berth[i].x && pos.first < berth[i].x + 4 && pos.second >= berth[i].y && pos.second < berth[i].y + 4){
                return {i, path};
            }
        }
        // 遍历当前位置的四个方向
        for (int i = 0; i < 4; i++) {
            int new_x = pos.first + dxs[i];
            int new_y = pos.second + dys[i];
            // 检查新坐标是否在矩阵范围内且是空地且未访问过
            if (new_x >= 1 && new_x < N && new_y >= 1 && new_y < N && !visited[new_x][new_y] && ch[new_x][new_y] != '*' && ch[new_x][new_y] != '#' && Collision(pos.first, pos.second, new_x, new_y, length, bot_id)) {
                vector<pair<int, int>> new_path = path;
                new_path.push_back({new_x, new_y});
                q.push({{new_x, new_y}, new_path});
                visited[new_x][new_y] = true;
            }
        }
    }
    return {-1, {}};
}

// 根据每paths里每一条路径的长度和目标物品的价值，用价值/距离来取最大的一条路径
vector<pair<int, int>> ChoosePath(vector<vector<pair<int, int>>>& paths) {
    vector<pair<int, int>> best_path;
    int best_value = 0;
    for (const auto& path : paths) {
        if(1000- id + gds_frame[path.back().first][path.back().second] < path.size()) continue;
        float value = 0;
        value = static_cast<double>(gds[path.back().first][path.back().second]) / path.size();
        if (value > best_value) {
            best_value = value;
            best_path = path;
        }
    }
    return best_path;
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
    // f1.open("data.txt",ios::out);
    Init();
    for(int zhen = 1; zhen <= 15000; zhen ++)
    {
        int id = Input();
        // f1 << "id: " << id << endl;
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
                // 每20帧重新计算一个机器人的新路径
                if(i == zhen % 20 && zhen > 20 && bot.zt == 1) {
                    bot.zt = 0;
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
            // f1 << "bot_num: " << bot_num << " status: " << bot.zt << endl;
            switch (bot.zt)
            {
            // 状态0：寻路物品
            case 0:{
                find_item:
                bot.clearPath();
                if(bot.bfs_fail_time > 20) {
                    break;
                }
                vector<vector<pair<int, int>>> paths = BFSItem({bot.x, bot.y}, bot_num);
                vector<pair<int, int>> path = ChoosePath(paths);
                if(path.empty()) {
                    bot.bfs_fail_time++;
                    break;
                }
                bot.zt = 1;
                bot.path = path;
                bot.current_index = 0;
                bot.mbx = path.back().first;
                bot.mby = path.back().second;
                // 正常找到路径，此帧可以继续走一步，不break
            }
            // 状态2：去物品点途中
            case 1:{

                // 物品被抢走
                if(gds[bot.mbx][bot.mby] == 0) {
                    bot.zt = 0;
                    goto find_item;
                }

                // 系统确认到达目的地
                if(bot.x == bot.mbx && bot.y == bot.mby){
                    bot.zt = 2;
                }
                // 还在路上
                else{
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
                    bot.value = gds[bot.path.back().first][bot.path.back().second];
                    gds[bot.path.back().first][bot.path.back().second] = 0;
                    bot.zt = 3;
                }
                else{
                    if(gds[bot.mbx][bot.mby] != 0) {
                        printf("get %d\n", bot_num);
                        break;
                    }
                    // 物品消失，重新寻路
                    else{
                        bot.zt = 0;
                        goto find_item;
                    }
                }
            }
            // 状态3：寻找泊位
            case 3:{
                find_berth:
                bot.clearPath();
                pair<int, vector<pair<int, int>>> path = BFSBerth({bot.x, bot.y}, bot_num);
                if(path.first == -1) {
                    break;
                }
                bot.zt = 4;
                bot.path = path.second;
                bot.current_index = 0;
                bot.target_berth = path.first;
                bot.mbx = path.second.back().first;
                bot.mby = path.second.back().second;
                // 寻到路径直接进入状态4
            }
            // 状态4：去泊位途中
            case 4:{
                // 系统确认到达目的地
                if(bot.x == bot.mbx && bot.y == bot.mby){
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

        // 每个泊位的船只数量（包括在路上和在泊位）
        int boat_num_in_berth[10] = {0};
        for(int i = 0; i < boat_num; i++){
            if(boat[i].pos==-1) continue;
            boat_num_in_berth[boat[i].pos]++;
        }

        for(int i = 0; i < boat_num; i++){
            // TODO: 如果在虚拟点，通过1.泊位价值+2.路径花费时间+3.装载速度 决定去哪个泊位
            if(boat[i].status == 1 && boat[i].pos== -1) {
                boat[i].num = 0;
                for(int j = 0; j < berth_num; j++){
                    if(berth[berth_priority[j].id].items.size()/boat_capacity >= boat_num_in_berth[berth_priority[j].id]){
                        boat_num_in_berth[berth_priority[j].id]++;
                        printf("ship %d %d\n", i, berth_priority[j].id);
                        break;
                    }
                }
            }
        }

        // 泊位装货，维持泊位信息与出发
        for(int i = 0; i < boat_num; i ++)
        {   
            // 在泊位
            if(boat[i].status == 1 && boat[i].pos != -1) {
                int target_berth = boat[i].pos;

                // 要结束了提前返回
                if(berth[target_berth].transport_time > 14990-id){
                    printf("go %d\n", i);
                    break;
                }
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
                    printf("go %d\n", i);
                    boat_num_in_berth[target_berth]--;
                    break;
                }
                // 没货了，船未满一半，且时间充裕，去别的泊位
                if(berth[target_berth].items.size() == 0) {
                    if(boat[i].num < boat_capacity*3/4 && berth[target_berth].transport_time < 14490 - id) {
                        for(int j = 0; j < berth_num; j++){
                            if(berth[berth_priority[j].id].items.size()/boat_capacity >= boat_num_in_berth[berth_priority[j].id]){
                                printf("ship %d %d\n", i, berth_priority[j].id);
                                boat_num_in_berth[berth_priority[j].id]++;
                                break;
                            }
                        }
                    }
                    else{
                        printf("go %d\n", i);
                        boat_num_in_berth[target_berth]--;
                        break;
                    }
                    break;
                }
            }
        }
        puts("OK");
        fflush(stdout);
    }
    // f1.close();
    return 0;
}
