#ifndef LIMORTON_ANTCOLONY_H_INCLUDED
#define LIMORTON_ANTCOLONY_H_INCLUDED

#include "limorton_Graph.h"
#include <vector>
//#include <deque>

using namespace std;

typedef double Excite;

//算法初始化参数结构体
struct InitPara{
    double phe_max = 10.0; //最大最小蚁群算法信息素上下限
    double phe_min = 1.0;
    double edgePeromone = phe_max;        //初始化边的信息素浓度

    double alpha = 2;           //信息素增强系数
    double beta = 4;            //距离信息偏重指数，，度量距离远近对选路径的影响，也称可见度，出现在算法的状态转移规则中
    double rou = 0.1;           //全局信息素挥发参数
    double rou_local = 0.1;     //局部信息素挥发参数
    double q_zero = 0.01;       //状态转移公式中的q0, 蚂蚁选择下条路径时随机选择的概率
    int loop_MAX = 50;         //最大循环次数
    //int ant_NUM = 100;          //蚂蚁个数，一般不多于网络结点数
};
static InitPara initParam;
static VideoNetGraph graph;

void Get_Graph(const VideoNetGraph& Graph);
/***辅助函数*************************/

///启发式:
//0- 图的评估 消费节点间距离计算——单源单目标点最短距离算法_部分dijkstra算法
//服务器的 个数 和 位置
//1- 连接的边数越多(最大输出流越大)优先级越高，返回优先级最高的结点，或全计算，加入概率中
//2- 只有一条边与之相连且不与消费结点相连的结点不会成为部署点,可直接跳过
//3- 事先评估图
//4- 若该结点连接消费结点，且 maxFlux < Need, 则必须在该结点放置服务器
//5- 消费节点间距离越大，蚂蚁寿命越短，需求越高，寿命越长
//6- 需求越高，亢奋度越高，寿命越长
int Set_Node_Priority();
bool Only_One_Edge(int nodeNum);
bool Flux_Over_Flow(int customNum);
//最近邻法选择下个结点
int Choose_Next_Node(int node, const vector<int>& visitedNode, int& minRent, int& band);

//最近邻法初始化全局最优路径
void Init_Global_Path(vector<int>& cost, vector<int>& band, const vector<int>& antLife);

/***辅助函数结束*********************/

//蚂蚁图类
class AntColonySystem{
private:
    //vector<int> deadAnt;        //走入死胡同的蚂蚁数量，nodeNum
    vector<double> nodePheromone;   //,nodeNum
public:
    Evaluater evaluate;         //对图的评估
    AntColonySystem(){}

    //初始化参数，初始化边和结点的信息素浓度等
    void Init_Parameter();

    //计算当前结点到下一结点的转移概率
    double Transition_Chance(int currentNode, int nextNode);

    //局部更新规则，单只蚂蚁经过某条路径过后，该路径信息素更新策略
    void Update_Local_Path_Rules(int currentNode, int nextNode, int antLife, int Lnn);

    //全局信息素更新，只更新最优路径上的信息素
    void Update_Global_Rules(const vector<int>& global_minPath, int global_minCost);

    //评估网络图指标
    void Estimate_Graph();
};
//蚂蚁类
class Ant{
private:
    AntColonySystem *antColony;

    int startNode;      //初始结点，共custNum种
    int currentNode;    //当前结点
    int currentCost;    //当前花费，链路租金
    int aimNeed;        // 实际需求带宽
    int currentBand_min;    //当前链路的最小带宽
    int antLife;       //寿命，最大行走步数
    Excite excited;    //亢奋度，表征释放信息素能力，需求越大，亢奋度越强
    vector<int> visitedNode; //禁忌表--1：表示未走过，0：已走过
    vector<int> route;  //当前路径
public:
    //Ant(){}
    //需要初始化寿命、当前花费、释放信息素能力、禁忌表、当前路径
    Ant(AntColonySystem *acs, int start, int life, Excite excite);

    //单蚂蚁开始搜索
    void Search(int Lnn);
    //选择下一个结点，//XXX优先走目前子图的边缘点(是不是成了Dijkstra算法了？类似反信息素了)和流量大的点
    int Choose_Next(int& minBand);
    //移动到下一个结点
    void Move_to_Next_Node(int nextNode);

    int Get_StartNode(){
        return startNode;
    }
    int Get_CurrentCost(){
        return currentCost;
    }
    int Get_CurrentBandmin(){
        return currentBand_min;
    }
    vector<int> Get_Route(){
        return route;
    }
};

//蚁群算法
void AntColonyAlogrithm(const VideoNetGraph& Graph);

//调试入口
void Debug_Into();

//直连
char* Direct_Connect();
#endif // LIMORTON_ANTCOLONY_H_INCLUDED
