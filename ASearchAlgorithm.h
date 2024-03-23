#ifndef ASEARCHALGORITHM_H
#define ASEARCHALGORITHM_H
#define MAP_SIZE 200
#define DIRECTIONS 4
#define MAX_PATH1 300
#define n 200
#define ini_robot_num 10
#define boat_num 5
#define good_num 100
#define N 210
#define berth_num 10
#define MAX_PATH 200

typedef struct Good
{
    int survive_time;
    int type;//判断属于哪个港口
    int x;
    int y;
    int value;
    float ratio;   //价值和距离的比值
    struct Good *next; 
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

typedef struct TraceNode{
    int x;
    int y;
    struct TraceNode *next;
} TraceNode;

typedef struct Node {
    Point point;
    int g, h, f;
    struct Node* parent;
} Node;

int heuristic(Point a, Point b);

int inList(Node* node, Node** list, int size);

void addToOpenList(Node* node,Node**openList);

void addToClosedList(Node* node,Node**closedList);

void removeFromOpenList(Node* node,Node**openList);

int isValidPoint(Point p, char map[200][201]);

Node* findPath(Point start, Point end, char map[200][201],Node**openList,Node** closedList,int *dx,int *dy);

void reconstructPath(Node* endNode,char* path,int *dx,int* dy,char* directions);

void freeAllNodes(Node** openList,Node** closedList);

int AStarAlgorithm(int start_x,int start_y,int end_x,int end_y,char map[200][201],char *path);

void ReverseTraverse(char *target,char*temp);

#endif