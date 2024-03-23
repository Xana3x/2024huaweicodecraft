#define _CRT_SECURE_NO_WARNINGS
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
//#include"ASearchAlgorithm.h"


#define MAP_SIZE 200
#define DIRECTIONS 4
#define MAX_PATH1 500
#define n 200
#define ini_robot_num 10
#define boat_num 5
#define good_num 100
#define N 210
#define berth_num 10

int domain[berth_num] = { 0 };

int large_five[5] = { -1 ,-1,-1,-1,-1 };
int small_five[5] = { -1 ,-1,-1,-1,-1 };

typedef struct Good
{
    int survive_time;
    int type;//判断属于哪个港口
    int x;
    int y;
    int value;
    float ratio;   //价值和距离的比值
    struct Good* next;
}Good;

typedef struct Robot
{
    int x, y, goods;   //是否存在货物  0为没有货物,1有货
    int status;   //用于判断是否是在清醒状态    0为昏迷  1为清醒
    int need_dirction;//用于判断目前是否需要安排指令  0为不需要指令   1为需要指令
    char robot_path[MAX_PATH1];  //指令数组
    int steps; //用于指示走了几步,然后在robot_path中导出指令,同时还可以辅助更改状态
    int berth;//属于那个港口
    int isuseless;
}Robot;

typedef struct Berth
{
    int x;
    int y;
    int transport_time;  //从虚拟点到这个停泊点时间（帧）
    int loading_speed;  //装船速度
    int current_goods_num;
}Berth;

typedef struct Boat
{
    int num, pos, status;  //num是当前承载货物的量，pos是当前目标，或者是停泊的点，虚拟点用-1代表，status代表状态0表示移动(运输)中 1表示正常运行状态(即装货状态或
    //运输完成状态)  2表示泊位外等待状态
}Boat;

typedef struct {
    int x;
    int y;
    int step;  // 记录步数
} QueueNode;


typedef struct {
    int x, y;
} Point;

typedef struct TraceNode {
    int x;
    int y;
    struct TraceNode* next;
} TraceNode;

typedef struct Node {
    Point point;
    int g, h, f;
    struct Node* parent;
} Node;


int robot_num = 10; //robot_num可修改


TraceNode* trace = NULL;

Good* head[10] = { NULL };    //头指针
Good* tail[10] = { NULL };    //尾指针

int id = 0;

Robot robot[ini_robot_num];

Berth berth[berth_num];

Boat boat[boat_num];

int connectivity[berth_num][berth_num] = { 0 };//通则为1  不通则为0

int max_time=0;

void getmaxtime()
{
    for (int i = 0; i < 10; i++)
    {
        if (berth[i].transport_time > max_time)
        {
            max_time = berth[i].transport_time;
        }
    }
}



int money, boat_capacity;    //id是轮次  boat_capacity是最大容龄
char ch[MAP_SIZE][MAP_SIZE + 1];//从ch[0][0]开始存储到199  199  //这是地图

void Init()
{
    for (int i = 0; i < n; i++)
        scanf("%s", ch[i]);
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        berth[i].current_goods_num = 0;
    }
    scanf("%d", &boat_capacity);  //所有的船的capacity是统一的
    char okk[100];
    scanf("%s", okk);
}





char path[200][200][MAX_PATH1];//从每个berth到每个点的路径


// 用于BFS搜索的四个方向
int dx[DIRECTIONS] = { 0, 0, -1, 1 };
int dy[DIRECTIONS] = { -1, 1, 0, 0 };
char directions[DIRECTIONS] = { '1','0','2','3' }; // 左右上下   0向右  1向左   2向上    3向下

// 检查点是否在地图内
int isInMap(int x, int y) {
    return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

int type[n][n];  //用于区分每个坐标的归类
int lenth[n][n];//用于快速获取每个坐标的到各自港口的路径长度

void InitTypeAndLenthArray()     //初始化数组
{
    memset(type, -1, sizeof(type));
    memset(lenth, -1, sizeof(type));
}

// BFS算法
void findNearestDock(char map[200][201], Berth* berth, char path[200][200][MAX_PATH1]) {
    int visited[MAP_SIZE][MAP_SIZE] = { 0 };//MAP_SIZE=200
    QueueNode queue[MAP_SIZE * MAP_SIZE];
    int front = 0, rear = 0;

    for (int i = 0; i < berth_num; i++)
    {
        int x = berth[i].x;
        int y = berth[i].y;
        queue[rear++] = (QueueNode){ x,y, 0 };
        visited[x][y] = 1;
        type[x][y] = i;
        lenth[x][y] = 0;
        strcpy(path[x][y], "\0");
    }

    while (front < rear) {
        int this_rear = rear;
        while (front < this_rear)
        {
            QueueNode current = queue[front++];

            for (int i = 0; i < DIRECTIONS; i++)
            {
                int newX = current.x + dx[i];
                int newY = current.y + dy[i];

                if (isInMap(newX, newY) && !visited[newX][newY] && map[newX][newY] != '#' && map[newX][newY] != '*')
                    //是空地而且在地图内   同时还要确认没被搜索过
                {
                    visited[newX][newY] = 1;
                    queue[rear].x = newX;
                    queue[rear].y = newY;
                    queue[rear].step = current.step + 1;
                    lenth[newX][newY] = current.step + 1;
                    type[newX][newY] = type[current.x][current.y];
                    strcpy(path[newX][newY], path[current.x][current.y]); // 就是这里有问题
                    path[newX][newY][current.step] = directions[i];
                    path[newX][newY][current.step + 1] = '\0';
                    rear++;
                }
            }
        }
    }
}

void Repositionberth(Berth* berth, char map[200][201])
{
    for (int i = 0; i < berth_num; i++)
    {
        int current_x = berth[i].x;
        int current_y = berth[i].y;
        if (map[current_x][current_y - 1] == '.' && map[current_x - 1][current_y] == '.') continue;    //左上
        if (map[current_x][current_y + 4] == '.' && map[current_x - 1][current_y + 3] == '.') {
            berth[i].y += 3;
            continue;
        }    //右上
        if (map[current_x + 4][current_y] == '.' && map[current_x + 3][current_y - 1] == '.') {
            berth[i].x += 3;
            continue;
        }     //左下
        if (map[current_x + 4][current_y + 3] == '.' && map[current_x + 3][current_y + 4] == '.') {
            berth[i].y += 3;
            berth[i].x += 3;
            continue;
        }     //右下
    }
}


void AddGood(Good* node)
{
    int type = node->type;
    /*if (type == 9)
    {
        printf("0\n");
    }*/
    if (tail[type] == NULL)
    {
        tail[type] = node;
        head[type] = node;
    }
    else
    {
        tail[type]->next = node;
        tail[type] = node;
    }
}

void DeleteGood()
{
    for (int i = 0; i < berth_num; i++)
    {
        Good* pointer = head[i];
        while (pointer->survive_time == 0)
        {
            free(pointer);
            pointer = pointer->next;
        }
        head[i] = pointer;
    }
}

int Input()
{
    scanf("%d%d", &id, &money);  //每一行一个scanf
    int num;     //新增货物数量
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        int goodtype = type[x][y];
        Good* good_node = (Good*)malloc(sizeof(Good));
        good_node->x = x;
        good_node->y = y;
        good_node->type = goodtype;
        good_node->survive_time = 1000;
        good_node->value = val;
        good_node->ratio = (float)val / lenth[x][y];
        good_node->next = NULL;
        if (goodtype != -1)
        {
            AddGood(good_node);
        }
        else {
            free(good_node);
        }

    }
    for (int i = 0; i < ini_robot_num; i++)
    {
        //robot[i].num = i;
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &robot[i].status);//goods代表是否携带货物0表示未携带物品。 
        //1表示携带物品。     sts是状态变量   0表示恢复状态   1表示正常运行状态
    }
    for (int i = 0; i < boat_num; i++)
        scanf("%d%d", &boat[i].status, &boat[i].pos);  //status 0表示移动(运输)中 
    //1表示正常运行状态(即装货状态或运输完成状态)       2表示泊位外等待状态
    //pos:表示目标泊位，如果目标泊位是虚拟点，则为-1
    char okk[100];
    scanf("%s", okk);
    return id;
}

Point findBestGood(int i)
{
    int berthid = robot[i].berth;
    if (head[berthid] == NULL)     //如果根本就没有节点
    {
        return (Point) { -1, -1 };
    }

    if (head[berthid] == tail[berthid])//如果两个指向同意一个节点
    {
        Good* pointer = head[berthid];
        head[berthid] = NULL;
        tail[berthid] = NULL;

        int x = pointer->x;
        int y = pointer->y;
        free(pointer);
        return (Point) { x, y };
    }

    float max = head[berthid]->ratio;
    Good* maxprepointer = head[berthid];
    Good* prepointer = head[berthid];

    while (prepointer->next != NULL)
    {
        if (prepointer->next->ratio > max && prepointer->next->survive_time > lenth[prepointer->next->x][prepointer->next->y])//最大而且可达
        {
            max = prepointer->next->ratio;
            maxprepointer = prepointer;
        }
        prepointer = prepointer->next;
    }
    if (max == head[berthid]->ratio)    //最大节点为第一个
    {
        Good* pointer = maxprepointer;
        head[berthid] = pointer->next;
        int x = pointer->x;
        int y = pointer->y;
        free(pointer);
        return (Point) { x, y };
    }
    if (maxprepointer->next == tail[berthid])
    {
        tail[berthid] = maxprepointer;
    }

    Good* pointer = maxprepointer->next;
    int x = pointer->x;
    int y = pointer->y;
    maxprepointer->next = maxprepointer->next->next;
    free(pointer);
    return (Point) { x, y };
}

void strrcpyAndModify(char* dest, const char* src) {  //这个函数相当重要，用于把从港口出发的路径转化为回到港口的路径
    int len = strlen(src);
    for (int i = 0; i < len; i++) {
        switch (src[len - i - 1])
        {
        case '0':
            dest[i] = '1';  // 0 -> 1
            break;
        case '1':
            dest[i] = '0';  // 1 -> 0
            break;
        case '2':
            dest[i] = '3';  // 2 -> 3
            break;
        case '3':
            dest[i] = '2';  // 3 -> 2
            break;
        }
    }
    dest[len] = '\0';  // 添加字符串结束符
}

void ArrangePath(int i)   //0就代表分配失败    1就代表安排成功
{
    int curX = robot[i].x;
    int curY = robot[i].y;
    if (robot[i].x == berth[robot[i].berth].x && robot[i].y == berth[robot[i].berth].y) //从berth出发
    {
        Point goodpoint = findBestGood(i);
        if (goodpoint.x == -1 && goodpoint.y == -1)  //说明此时没有货物，就在原地保持不动
        {
            robot[i].need_dirction = 1;
            robot[i].robot_path[0] = '\0';   //把路径封死
            //robot[i].steps = 0;
            //return 0;
        }
        else {  //此时说明安排成功
            strcpy(robot[i].robot_path, path[goodpoint.x][goodpoint.y]);
            robot[i].need_dirction = 0;
        }


    }
    else {//拿货了，回到berth
        strrcpyAndModify(robot[i].robot_path, path[robot[i].x][robot[i].y]);
    }
    robot[i].steps = 0;
    //return 1;
}

// ToDo:机器人控制

void ArrangeRobots(int i)
{
    if (robot[i].status == 1)  //此时机器人状态正常  //现在来看多余了
    {
        if (robot[i].robot_path[robot[i].steps] == '\0') //走完了安排的路线
        {
            if (robot[i].need_dirction == 0) {//说明此时不在等待状态
                if (robot[i].x == berth[robot[i].berth].x && robot[i].y == berth[robot[i].berth].y)   //说明到达了港口
                {
                    printf("pull %d\n", i);
                    ArrangePath(i);
                    robot[i].goods = 0;
                    berth[robot[i].berth].current_goods_num++;

                }
                else//到达了货物处
                {
                    printf("get %d\n", i);
                    ArrangePath(i);
                    robot[i].goods = 1;
                }
            }
            else {  //在等待状态   就只安排路径而不做状态修改
                ArrangePath(i);
            }

        }
    }

}


void UpdataGoods()
{
    for (int i = 0; i < berth_num; i++)
    {
        Good* pointer = head[i];
        if (pointer == NULL)continue;  //说明此时没有货物在这个地点
        else {
            while (pointer->survive_time == 1 && pointer->next)
            {
                Good* temp = pointer;
                pointer = pointer->next;
                free(temp);
            }
            head[i] = pointer;
            if (head[i] == NULL)tail[i] = NULL;
            while (pointer != NULL)
            {
                pointer->survive_time--;
                pointer = pointer->next;
            }
        }

    }
    //DeleteGood();   如果使用deletegood的话会要遍历很多遍，所以不如在这里直接删除
}

int PickOutBerth()
{
    int boat_berth_pair[berth_num];
    //memset(boat_berth_pair,1,sizeof(boat_berth_pair));
    for (int i = 0; i < berth_num; i++)    //只有这样才能正确的置为1   memset只能用于char型
    {
        boat_berth_pair[i] = 1;
    }
    for (int i = 0; i < boat_num; i++)
    {
        if (boat[i].pos != -1)boat_berth_pair[boat[i].pos] = 0;  //需要在送出后立马把boat的pos设为-1
    }
    int pickout;
    int max = 0;
    for (int j = 0; j < berth_num; j++)
    {
        if (berth[j].current_goods_num >= max && boat_berth_pair[j] == 1)
        {
            pickout = j;
            max = berth[j].current_goods_num;
        }
    }
    return pickout;
}

void SendBoat(int i)
{
    int pickedberth = PickOutBerth();
    boat[i].pos = pickedberth;
    printf("ship %d %d\n", i, pickedberth);
}

void ShipGo(int i)
{
    printf("go %d\n", i);
}

void UpdataShips()
{
    for (int i = 0; i < boat_num; i++)
    {
        if (boat[i].status == 1)
        {
            if (boat[i].pos != -1)
            {   //此时为在港口外停泊
                int curberth = boat[i].pos;
                int carriedgoodnum = berth[curberth].loading_speed > berth[curberth].current_goods_num ?
                    berth[curberth].current_goods_num : berth[curberth].loading_speed;
                int dif = boat_capacity - boat[i].num;
                carriedgoodnum = dif > carriedgoodnum ? carriedgoodnum : dif;
                berth[curberth].current_goods_num -= carriedgoodnum;
                boat[i].num += carriedgoodnum;
            }
        }

    }
}


void Update()
{
    UpdataShips();
    UpdataGoods();
}

void ManipulateBoats(int i)
{
    if (id == (15000 - max_time-1) )
    {
        ShipGo(i);
        return;
    }
    if (boat[i].pos != -1)
    {
        if (boat[i].num == boat_capacity)//满了  去卸货
        {
            ShipGo(i);
            boat[i].num = 0;
            boat[i].status = 0;  //status会自己更新
            boat[i].pos = -1;
            return;
        }
        if (berth[boat[i].pos].current_goods_num == 0)
        {
            SendBoat(i);
            boat[i].status = 0;
        }
    }

    


    if (boat[i].pos == -1 && boat[i].status == 1)//在虚拟点   就回来
    {
        SendBoat(i);
        boat[i].status = 0;
    }

}

void BoatAssign()
{
    printf("ship 0 1\nship 1 3\nship 2 5\nship 3 7\nship 4 9\n");
    for (int i = 0; i < boat_num; i++)
    {
        boat[i].status = 0;
        boat[i].pos = i * 2 + 1;
        boat[i].num = 0;
    }
}

void removeElement(Robot* robot, int index, int size)  //这个目前用不到了
{
    for (int i = index; i < size - 1; i++)
    {
        robot[i] = robot[i + 1];
    }
}

void CleanUpRobots()//用于判断机器人是否在一个密闭空间中,如果是就永远不操作它
{
    for (int i = 0; i < boat_num; i++)
    {
        int x = robot[i].x;
        int y = robot[i].y;
        robot[i].isuseless = 0;
        if (type[x][y] == -1)
        {
            // removeElement(robot,i,robot_num);
            // robot_num--;
            robot[i].isuseless = 1;
        }
    }
}

int FindLeftArray(int* arr, int size) {   //有多少个没被分配
    int count = 0;
    for (int i = 0; i < size; i++)
    {
        if (arr[i] == -1)count++;
    }
    return count;
}

void StuffArray(int* arr1, int* arr2, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (arr2[i] == -1)*(arr1++) = i;
    }
}

float EuclideanDistance(int x1, int y1, int x2, int y2) {
    int dx = x1 - x2;
    int dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}


void RobotAssign1()
{
    int max_berth1 = large_five[0];
    int max_berth2 = large_five[1];
    int middle_six[6] = { large_five[2],large_five[3],large_five[4],small_five[4],small_five[3],small_five[2] };
    int max_berth[2] = { max_berth1,max_berth2 };
    int robot_assign[ini_robot_num];
    memset(robot_assign, -1, ini_robot_num * sizeof(int));
    for (int j = 0; j < robot_num; j++)
    {
        if (robot[j].isuseless == 1)robot_assign[j] = 100;
    }

    for (int i = 0; i < 2; i++)
    {
        int possessed_robot = 0;
        for (int j = 0; j < robot_num; j++)
        {
            if (robot_assign[j] == -1 && type[robot[j].x][robot[j].y] == max_berth[i])
            {

                int curX = robot[j].x;
                int curY = robot[j].y;
                robot_assign[j] = max_berth[i];
                robot[j].berth = max_berth[i];
                strrcpyAndModify(robot[j].robot_path, path[curX][curY]);
                robot[j].steps = 0;
                robot[j].need_dirction = 0;
                robot[j].goods = 0;
                robot[j].status = 1;
                possessed_robot++;
            }
            if (possessed_robot == 2)break;

        }
        if (possessed_robot < 2)
        {
            Point end = { berth[max_berth[i]].x,berth[max_berth[i]].y };
            int dif = 2 - possessed_robot;
            for (int j = 0; j < dif; j++)
            {
                int min = 65535;
                int picked = -1;
                for (int k = 0; k < robot_num; k++)
                {
                    if (robot_assign[k] == -1 && connectivity[type[robot[k].x][robot[k].y]][max_berth[i]])
                    {
                        int curx = robot[k].x;
                        int cury = robot[k].y;
                        Point start = { curx,cury };
                        int dis = heuristic(start, end);
                        if (dis <= min)
                        {
                            picked = k;
                            min = dis;
                        }
                    }
                }
                if (picked != -1)
                {
                    int curX = robot[picked].x;
                    int curY = robot[picked].y;
                    robot_assign[picked] = max_berth[i];
                    robot[picked].berth = max_berth[i];
                    AStarAlgorithm(curX, curY, berth[max_berth[i]].x, berth[max_berth[i]].y, ch, robot[picked].robot_path);
                    robot[picked].steps = 0;
                    robot[picked].need_dirction = 0;
                    robot[picked].goods = 0;
                    robot[picked].status = 1;
                    possessed_robot++;
                }
            }
        }

    }

    for (int j = 0; j < 6; j++)
    {
        int curberth = middle_six[j];
        int x = berth[curberth].x;
        int y = berth[curberth].y;
        Point end = { x,y };
        int min = 16000;
        int choose = -2;
        for (int i = 0; i < robot_num; i++)
        {
            int curX = robot[i].x;
            int curY = robot[i].y;
            int domain = type[curX][curY];

            if (curberth == domain && min >= lenth[curX][curY])
            {
                choose = i;
                min = lenth[curX][curY];
            }

        }

        if (choose != -2)  //选到了一个机器人
        {
            int curX = robot[choose].x;
            int curY = robot[choose].y;
            robot_assign[choose] = j;
            robot[choose].berth = j;
            strrcpyAndModify(robot[choose].robot_path, path[curX][curY]);
            robot[choose].steps = 0;
            robot[choose].need_dirction = 0;
            robot[choose].goods = 0;
            robot[choose].status = 1;
        }
        else
        {
            for (int k = 0; k < robot_num; k++)
            {
                if (robot_assign[k] == -1 && connectivity[type[robot[k].x][robot[k].y]][curberth])
                {
                    int curx = robot[k].x;
                    int cury = robot[k].y;
                    Point start = { curx,cury };
                    int dis = heuristic(start, end);
                    if (dis <= min)
                    {
                        choose = k;
                        min = dis;
                    }
                }
            }
            if (choose != -2)
            {
                int curX = robot[choose].x;
                int curY = robot[choose].y;
                robot_assign[choose] = j;
                robot[choose].berth = j;
                AStarAlgorithm(curX, curY, berth[j].x, berth[j].y, ch, robot[choose].robot_path);
                robot[choose].steps = 0;
                robot[choose].need_dirction = 0;
                robot[choose].goods = 0;
                robot[choose].status = 1;
            }
        }
    }
}


void RobotAssign()
{
    int berth_assign[berth_num];
    int robot_assign[ini_robot_num];
    memset(berth_assign, -1, berth_num * sizeof(int));
    memset(robot_assign, -1, ini_robot_num * sizeof(int));
    for (int i = 0; i < robot_num; i++)
    {
        if (robot[i].isuseless == 1)robot_assign[i] = 100;//就可以看成已经分配了
    }
    for (int j = 0; j < berth_num; j++)
    {
        int min = 16000;
        int choose = -2;
        for (int i = 0; i < robot_num; i++)
        {
            int curX = robot[i].x;
            int curY = robot[i].y;
            int domain = type[curX][curY];

            if (j == domain && min >= lenth[curX][curY])
            {
                choose = i;
                min = lenth[curX][curY];
            }

        }

        if (choose != -2)  //选到了一个机器人
        {
            int curX = robot[choose].x;
            int curY = robot[choose].y;
            berth_assign[j] = choose;
            robot_assign[choose] = j;
            robot[choose].berth = j;
            strrcpyAndModify(robot[choose].robot_path, path[curX][curY]);
            robot[choose].steps = 0;
            robot[choose].need_dirction = 0;
            robot[choose].goods = 0;
            robot[choose].status = 1;
        }
    }
    int left_berth = FindLeftArray(berth_assign, berth_num);
    int left_robot = FindLeftArray(robot_assign, robot_num);
    int* left_berth_arr = (int*)malloc(left_berth * sizeof(int));
    int* left_robot_arr = (int*)malloc(left_robot * sizeof(int));
    StuffArray(left_berth_arr, berth_assign, berth_num);
    StuffArray(left_robot_arr, robot_assign, robot_num);
    for (int i = 0; i < left_robot; i++)
    {
        int min = 65535;
        int this = 65535;
        for (int j = 0; j < left_berth; j++)
        {
            int robot_x = robot[left_robot_arr[i]].x;
            int robot_y = robot[left_robot_arr[i]].y;
            int berth_x = berth[left_berth_arr[j]].x;
            int berth_y = berth[left_berth_arr[j]].y;
            int dis = EuclideanDistance(robot_x, robot_y, berth_x, berth_y);
            if (dis <= min && berth_assign[left_berth_arr[j]] == -1 && connectivity[type[robot_x][robot_y]][left_berth_arr[j]])
            {
                this = left_berth_arr[j];
                min = dis;
            }
        }
        int choosed_berth = this;
        int choosed_robot = left_robot_arr[i];

        int curX = robot[choosed_robot].x;
        int curY = robot[choosed_robot].y;

        int succ = AStarAlgorithm(robot[choosed_robot].x, robot[choosed_robot].y, berth[choosed_berth].x, berth[choosed_berth].y, ch, robot[choosed_robot].robot_path);
        if (succ == 0)  //这是成功了
        {
            robot[choosed_robot].berth = choosed_berth;
            berth_assign[choosed_berth] = choosed_robot;
            robot_assign[choosed_robot] = choosed_berth;
            robot[choosed_robot].steps = 0;
            robot[choosed_robot].need_dirction = 0;
            robot[choosed_robot].goods = 0;
            robot[choosed_robot].status = 1;
        }
        else {  //失败  //先简单安排
            exit(55555);
        }

    }

    free(left_berth_arr);
    free(left_robot_arr);


}

void RobotAssign2()
{
    int berth_assign[berth_num];
    int robot_assign[ini_robot_num];
    memset(berth_assign, -1, berth_num * sizeof(int));
    memset(robot_assign, -1, ini_robot_num * sizeof(int));
    for (int i = 0; i < robot_num; i++)
    {
        if (robot[i].isuseless == 1)robot_assign[i] = 100;//就可以看成已经分配了
    }
    for (int j = 0; j < berth_num; j++)
    {
        int min = 16000;
        int choose = -2;
        for (int i = 0; i < robot_num; i++)
        {
            int curX = robot[i].x;
            int curY = robot[i].y;
            int domain = type[curX][curY];

            if (j == domain && min >= lenth[curX][curY])
            {
                choose = i;
                min = lenth[curX][curY];
            }

        }

        if (choose != -2)  //选到了一个机器人
        {
            int curX = robot[choose].x;
            int curY = robot[choose].y;
            berth_assign[j] = choose;
            robot_assign[choose] = j;
            robot[choose].berth = j;
            strrcpyAndModify(robot[choose].robot_path, path[curX][curY]);
            robot[choose].steps = 0;
            robot[choose].need_dirction = 0;
            robot[choose].goods = 0;
            robot[choose].status = 1;
        }
    }
    int left_berth = FindLeftArray(berth_assign, berth_num);
    int left_robot = FindLeftArray(robot_assign, robot_num);
    int* left_berth_arr = (int*)malloc(left_berth * sizeof(int));
    int* left_robot_arr = (int*)malloc(left_robot * sizeof(int));
    StuffArray(left_berth_arr, berth_assign, berth_num);
    StuffArray(left_robot_arr, robot_assign, robot_num);
    for (int i = 0; i < left_robot; i++)
    {
        int min = 65535;
        int this = 65535;
        for (int j = 0; j < left_berth; j++)
        {
            int robot_x = robot[left_robot_arr[i]].x;
            int robot_y = robot[left_robot_arr[i]].y;
            int berth_x = berth[left_berth_arr[j]].x;
            int berth_y = berth[left_berth_arr[j]].y;
            int dis = EuclideanDistance(robot_x, robot_y, berth_x, berth_y);
            if (dis <= min && berth_assign[left_berth_arr[j]] == -1 && connectivity[type[robot_x][robot_y]][left_berth_arr[j]])
            {
                this = left_berth_arr[j];
                min = dis;
            }
        }
        int choosed_berth = this;
        int choosed_robot = left_robot_arr[i];

        int curX = robot[choosed_robot].x;
        int curY = robot[choosed_robot].y;

        int succ = AStarAlgorithm(robot[choosed_robot].x, robot[choosed_robot].y, berth[choosed_berth].x, berth[choosed_berth].y, ch, robot[choosed_robot].robot_path);
        if (succ == 0)  //这是成功了
        {
            robot[choosed_robot].berth = choosed_berth;
            berth_assign[choosed_berth] = choosed_robot;
            robot_assign[choosed_robot] = choosed_berth;
            robot[choosed_robot].steps = 0;
            robot[choosed_robot].need_dirction = 0;
            robot[choosed_robot].goods = 0;
            robot[choosed_robot].status = 1;
        }
        else {  //失败  //先简单安排
            exit(55555);
        }

    }

    free(left_berth_arr);
    free(left_robot_arr);

    int range[10] = { large_five[0],large_five[1],large_five[2],large_five[3],large_five[4],small_five[4],small_five[3],small_five[2],small_five[1],small_five[0] };
    int front = 0;
    int rear = 9;
    while (front <= 1)
    {
        if (berth_assign[range[rear]] == -1)
        {
            rear--;
            continue;
        }
        if (berth_assign[range[front]] == -1)
        {
            int robotid = berth_assign[range[rear]];
            int berthid = range[front];
            if (connectivity[type[robot[robotid].x][robot[robotid].y]][berthid] == 1)  //表明是相连的
            {
                robot[robotid].berth = range[front];
                //用astar算法
                int curX = robot[robotid].x;
                int curY = robot[robotid].y;
                AStarAlgorithm(curX, curY, berth[berthid].x, berth[berthid].y, ch, robot[robotid].robot_path);
                berth_assign[berthid] = robotid;
                robot_assign[robotid] = berthid;
            }
        }
        int robotid = berth_assign[range[rear]];
        int berthid = range[front];
        if (connectivity[type[robot[robotid].x][robot[robotid].y]][berthid] == 1)  //表明是相连的
        {
            robot[robotid].berth = range[front];
            //用astar算法
            int curX = robot[robotid].x;
            int curY = robot[robotid].y;
            AStarAlgorithm(curX, curY, berth[berthid].x, berth[berthid].y, ch, robot[robotid].robot_path);
            berth_assign[berthid] = robotid;
            robot_assign[robotid] = berthid;
        }
        rear--;
        front++;
    }

}

void Assign()
{
    BoatAssign();
    RobotAssign();
}

int NotInTrace(int x, int y)
{
    if (trace == NULL)return 1;
    TraceNode* poiner = trace;
    while (poiner != NULL)
    {
        if (poiner->x == x && poiner->y == y)return 0;
        poiner = poiner->next;
    }
    return 1;
}

int movex[DIRECTIONS] = { 0,0,-1,1 };
int movey[DIRECTIONS] = { 1,-1,0,0 };


void insert_chars(char* str, int pos, char char1, char char2) {
    int len = strlen(str);
    if (pos < 0 || pos > len) {
        printf("Invalid position!\n");
        return;
    }

    // 将字符串从插入位置开始的所有字符后移两位
    memmove(str + pos + 2, str + pos, len - pos + 1);

    // 在插入位置插入两个字符
    str[pos] = char1;
    str[pos + 1] = char2;
}


void RobotsWalk(int i,char *cache)
{
    int curX = robot[i].x;
    int curY = robot[i].y;
    int nextX = curX + movex[robot[i].robot_path[robot[i].steps] - '0'];
    int nextY = curY + movey[robot[i].robot_path[robot[i].steps] - '0'];
    if (NotInTrace(curX, curY) && NotInTrace(nextX, nextY))//如果当前位置和下一位置都不在trace中
        //这里可以改来避免那种狭路相逢的局面
    {
        TraceNode* newnode = (TraceNode*)malloc(sizeof(TraceNode));
        newnode->x = curX;
        newnode->y = curY;
        newnode->next = trace;
        trace = newnode;
        TraceNode* newnode1 = (TraceNode*)malloc(sizeof(TraceNode));
        newnode1->x = nextX;
        newnode1->y = nextY;
        newnode1->next = trace;
        trace = newnode1;
        int flag = 0;
        for (int j = 0; j <10 ; j++)
        {
            if (nextX == robot[j].x && nextY == robot[j].y)
            {
                flag = 1;
            }
        }
        if (!flag)
        {
            printf("move %d %c\n", i, robot[i].robot_path[robot[i].steps]);
        }
        else {
            char temp[40] = "";
            sprintf(temp,"move %d %c\n", i, robot[i].robot_path[robot[i].steps]);
            strcat(cache,temp);
        }
        
        robot[i].steps++;
    }
    else if (!NotInTrace(curX, curY) && !NotInTrace(nextX, nextY))
    {
       
        char append=0;
        char append1=0;
        char nextmove = robot[i].robot_path[robot[i].steps];
        char lastmove= robot[i].robot_path[robot[i].steps-1];
        switch (lastmove) {
        case '0':lastmove = '1'; break;
        case '1':lastmove = '0'; break;
        case '2':lastmove = '3'; break;
        case '3':lastmove = '2'; break;
        }
        for (int j = 0; j < 4; j++)
        {
            if (nextmove == directions[j]|| lastmove== directions[j])continue;
            int nextx = robot[i].x + dx[j];
            int nexty = robot[i].y + dy[j];
            if (ch[nextx][nexty] != '*' && ch[nextx][nexty] != '#')
            {
                append = directions[j];
                switch (append) {
                case '0':append1 = '1'; break;
                case '1':append1 = '0'; break;
                case '2':append1 = '3'; break;
                case '3':append1 = '2'; break;
                }
                break;
            }
        }
        
        if (append == 0)
        {

            char back = 0;
            switch (robot[i].robot_path[robot[i].steps - 1])
            {
            case '0':back = '1'; break;
            case '1':back = '0'; break;
            case '2':back = '3'; break;
            case '3':back = '2'; break;
            }


            TraceNode* newnode = (TraceNode*)malloc(sizeof(TraceNode));
            newnode->x = curX + movex[back - '0'];
            newnode->y = curY + movey[back - '0'];
            newnode->next = trace;
            trace = newnode;
            printf("move %d %c\n", i, back);
            robot[i].steps--;
        }
        else{
            insert_chars(robot[i].robot_path, robot[i].steps, append, append1);
            printf("move %d %c\n", i, robot[i].robot_path[robot[i].steps]);
            robot[i].steps++;
        }
        
    }
    if (!NotInTrace(curX, curY) && NotInTrace(nextX, nextY))
    {
        TraceNode* newnode = (TraceNode*)malloc(sizeof(TraceNode));
        newnode->x = curX;
        newnode->y = curY;
        newnode->next = trace;
        trace = newnode;
        TraceNode* newnode1 = (TraceNode*)malloc(sizeof(TraceNode));
        newnode1->x = nextX;
        newnode1->y = nextY;
        newnode1->next = trace;
        trace = newnode1;
        int flag = 0;
        for (int j = 0; j < 10; j++)
        {
            if (nextX == robot[j].x && nextY == robot[j].y)
            {
                flag = 1;
            }
        }
        if (!flag)
        {
            printf("move %d %c\n", i, robot[i].robot_path[robot[i].steps]);
        }
        else {
            char temp[40] = "";
            sprintf(temp, "move %d %c\n", i, robot[i].robot_path[robot[i].steps]);
            strcat(cache, temp);
        }

        robot[i].steps++;
    }
}

void freeTrace()
{
    while (trace != NULL)
    {
        TraceNode* temp = trace;
        trace = trace->next;
        free(temp);
    }
}



#define MAP_SIZE 200
#define DIRECTIONS 4
int openListSize = 0;
int closedListSize = 0;


int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

int inList(Node* node, Node** list, int size) {
    for (int i = 0; i < size; i++) {
        if (list[i]->point.x == node->point.x && list[i]->point.y == node->point.y) {
            return i;
        }
    }
    return -1;
}//找点

void addToOpenList(Node* node, Node** openList) {   //大问题   这里直接openlistsize变得巨大  不知道为什么
    openList[openListSize] = node;
    openListSize++;
}

void addToClosedList(Node* node, Node** closedList) {
    closedList[closedListSize] = node;
    closedListSize++;
}

Node* lowestFInOpenList(Node** openList) {   //这个报错
    /*if (openList == 1)
    {
        return *openList;
    }*/
    Node* lowestNode = openList[0];
    for (int i = 1; i < openListSize; i++) {
        if (openList[i]->f < lowestNode->f) {
            lowestNode = openList[i];
        }
    }
    return lowestNode;
}

void removeFromOpenList(Node* node, Node** openList) {
    int index = inList(node, openList, openListSize);
    if (index != -1) {
        for (int i = index; i < openListSize - 1; i++) {
            openList[i] = openList[i + 1];
        }
        openListSize--;
    }
}

int isValidPoint(Point p, char map[200][201])
{
    return (p.x >= 0 && p.x < MAP_SIZE && p.y >= 0 && p.y < MAP_SIZE && map[p.x][p.y] != '#' && map[p.x][p.y] != '*');
}

Node* findPath(Point start, Point end, char map[200][201], Node** openList, Node** closedList, int* dx, int* dy)
{
    Node* startNode = (Node*)malloc(sizeof(Node));   //其实应该判断是否分配成功的，但我就直接不判断了
    startNode->point = start;
    startNode->g = 0;
    startNode->h = heuristic(start, end);
    startNode->f = startNode->g + startNode->h;
    startNode->parent = NULL;

    addToOpenList(startNode, openList);

    while (openListSize > 0) {
        Node* currentNode = lowestFInOpenList(openList);
        if (currentNode->point.x == end.x && currentNode->point.y == end.y)
        {
            return currentNode;
        }

        removeFromOpenList(currentNode, openList);
        //liststart++;
        addToClosedList(currentNode, closedList);

        for (int i = 0; i < DIRECTIONS; i++) {//四个方向
            Point neighborPoint = { currentNode->point.x + dx[i], currentNode->point.y + dy[i] };
            if (!isValidPoint(neighborPoint, map)) continue;  //无效点直接跳过

            Node* neighborNode = (Node*)malloc(sizeof(Node));
            neighborNode->point = neighborPoint;
            neighborNode->g = currentNode->g + 1;
            neighborNode->h = heuristic(neighborPoint, end);
            neighborNode->f = neighborNode->g + neighborNode->h;
            neighborNode->parent = currentNode;

            if (inList(neighborNode, closedList, closedListSize) != -1) { //在closelist中了
                free(neighborNode);
                continue;
            }

            int openIndex = inList(neighborNode, openList, openListSize);//找在openlist中的序号
            if (openIndex == -1)
            {
                addToOpenList(neighborNode, openList);
            }
            else {
                if (neighborNode->g < openList[openIndex]->g)
                {
                    openList[openIndex]->g = neighborNode->g;
                    openList[openIndex]->f = neighborNode->f;
                    openList[openIndex]->parent = currentNode;
                }
                free(neighborNode);
            }
        }
    }
    return NULL;
}

void reconstructPath(Node* endNode, char* path, int* dx, int* dy, char* directions)
{
    Node* currentNode = endNode;
    while (currentNode->parent != NULL)
    {
        int dirIndex = -1;
        for (int i = 0; i < DIRECTIONS; i++)
        {
            if (currentNode->point.x - currentNode->parent->point.x == dx[i] &&
                currentNode->point.y - currentNode->parent->point.y == dy[i])
            {
                dirIndex = i;
                break;
            }
        }
        if (dirIndex != -1)
        {
            *(path++) = directions[dirIndex];
        }

        currentNode = currentNode->parent;

    }
    *path = '\0';
}

void freeAllNodes(Node** openList, Node** closedList)
{
    for (int i = 0; i < openListSize; i++) {
        free(openList[i]);
    }
    for (int i = 0; i < closedListSize; i++) {
        free(closedList[i]);
    }
}

void ReverseTraverse(char* target, char* temp)
{
    int len = strlen(temp);
    for (int i = 0; i < len; i++)
    {
        target[i] = temp[len - i - 1];
    }
    target[len] = '\0';
}



int AStarAlgorithm(int start_x, int start_y, int end_x, int end_y, char map[200][201], char* path)
{
    Node** openList = (Node**)malloc(MAP_SIZE * MAP_SIZE * sizeof(Node*));
    Node** closedList = (Node**)malloc(MAP_SIZE * MAP_SIZE * sizeof(Node*));

    openListSize = 0;
    closedListSize = 0;

    int dx1[DIRECTIONS] = { 0, 0, -1, 1 };
    int dy1[DIRECTIONS] = { -1, 1, 0, 0 };
    char directions1[DIRECTIONS] = { '1', '0', '2', '3' };  //还是得用这个形式，不然0就被当做终止符了
    //char directions1[DIRECTIONS] = {1, 0, 2, 3};  //0向右 1向左 2向上 3向下
    Point start_point = { start_x,start_y };
    Point end_point = { end_x,end_y };
    char temp_path[MAX_PATH1];

    Node* result = findPath(start_point, end_point, map, openList, closedList, dx1, dy1);
    if (result == NULL)
    {
        freeAllNodes(openList, closedList);
        free(openList);
        free(closedList);
        return -1;
    }
    reconstructPath(result, temp_path, dx1, dy1, directions1);
    ReverseTraverse(path, temp_path);
    freeAllNodes(openList, closedList);
    free(openList);
    free(closedList);
    return 0;
}

void printfmap()
{
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            if (type[i][j] == -1)printf("0");
            else { printf("%d", type[i][j]); }

        }
        printf("\n");
    }
}


void checkmap()
{
    int count = 0;
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            if (type[i][j] != -1)count++;
        }
    }
    printf("%d\n", count);
}

void connectivityCheck()
{
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            int thistype = type[i][j];
            if (thistype == -1)continue;
            for (int d = 0; d < 4; d++)
            {
                int curX = i + dx[d];
                int curY = j + dy[d];
                int neighbortype = type[curX][curY];
                if (neighbortype == -1)continue;
                else {
                    connectivity[thistype][neighbortype] = 1;
                    connectivity[neighbortype][thistype] = 1;
                }
            }
        }
    }
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            for (int l = 0; l < 10; l++)
            {
                if (connectivity[j][l] == 1)
                {
                    connectivity[i][j] = 1;
                    connectivity[j][i] = 1;
                }
            }
        }
    }
}




void describedomain()
{
    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            int thistype = type[i][j];
            domain[thistype]++;
        }
    }
    int picked[10] = { 0 };

    for (int i = 0; i < 5; i++)
    {
        int max = 0;
        int pickedberthid = -1;
        for (int j = 0; j < 10; j++)
        {
            if (domain[j] > max && picked[j] == 0)
            {
                max = domain[j];
                pickedberthid = j;
            }
        }
        picked[pickedberthid] = 1;
        large_five[i] = pickedberthid;
    }
    int k = 0;
    for (int j = 0; j < 10; j++)
    {
        if (picked[j] == 0)
        {
            picked[j] = 1;
            small_five[k++] = j;

        }
    }
    //帮我写个排序，根据domain的大小，把small_five排序  
    for (int i = 0; i < 4; i++)
    {
        for (int j = i + 1; j < 5; j++)
        {
            if (domain[small_five[i]] > domain[small_five[j]])
            {
                int temp = small_five[i];
                small_five[i] = small_five[j];
                small_five[j] = temp;
            }
        }
    }


}


//char pathbetweenberth[berth_num][berth_num][MAX_PATH1];
//
//void buidpathbetweenberth()
//{
//    int big[5] = { -1,-1,-1,-1,-1 };
//    int small[5] = { -1,-1,-1,-1,-1 };
//    for (int i = 0; i < 5;i++)
//    {
//        Point start = { berth[large_five[i]].x,berth[large_five[i]].y };
//        int dis = 65535;
//        int pick = -1;
//        for (int j = 0; j < 5; j++)
//        {
//            Point end = { berth[small_five[j]].x,berth[small_five[j]].y };
//            if (small[j] == -1 && heuristic(start, end) < dis && connectivity[i][j] == 1)
//            {
//                dis = heuristic(start, end);
//                pick
//            }
//        }
//    }
//}


int main()
{

    /* double cpu_time_used;
    clock_t start, end;*/
    InitTypeAndLenthArray();
    Init();  //初始化接收函数
    //fprintf(stderr,"Init finished\n");
    //start = clock();
    Repositionberth(berth, ch);   //重新选择berth的位置
    findNearestDock(ch, berth, path);   //构建路径
    //printfmap();
    connectivityCheck();
    describedomain();
    getmaxtime();

    //checkmap();

    printf("OK\n");
    fflush(stdout);//开始进入循环

    //这是第一帧
    Input();
    CleanUpRobots();//清洗机器人数量和结构
    Assign();
    puts("OK");   //puts会自动添加换行
    fflush(stdout);
    //end = clock();
    //cpu_time_used = ((double)(end - start)) * 1000.0 / CLOCKS_PER_SEC;

    //// 打印程序运行时间
    //printf("程序运行时间: %f 毫秒\n", cpu_time_used);
    //目前来看花费时间122ms，可以接受
    while (id <= 15000)
    {
        char cache[200] = {'\0'};
        Input();   //input里面执行插入
       
        for (int i = 0; i < robot_num; i++)
        {   //机器人控制指令i
            if (robot[i].status == 1 && robot[i].isuseless == 0)  //运行逻辑最后一点点问题
            {

                ArrangeRobots(i);

                if (robot[i].need_dirction == 0)
                {
                    RobotsWalk(i,cache);
                }

            }
        }
        printf("%s", cache);
        freeTrace();
        Update();//对商品,船的商品数量执行更新


        for (int i = 0; i < boat_num; i++)
        {
            ManipulateBoats(i); //对船的控制
        }
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
