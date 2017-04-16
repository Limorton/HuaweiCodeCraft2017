#include "limorton_antColony.h"
#include "limorton_Graph.h"
#include <vector>
#include <deque>
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <iostream>

using namespace std;

#define lim_debug 0

/*辅助函数*/
int Set_Node_Priority(){
    int bestNode = 0;
    ////coding
    return bestNode;
}

//判断是否是单边结点
bool Only_One_Edge(int nodeNum){
    return ((graph.adjList[nodeNum].edgeAttached == 1)? true : false);
}

bool Flux_Over_Flow(int customNum){
    return (graph.adjList[graph.customerInfo[customNum].toNodeID].maxFlux < graph.customerInfo[customNum].bandNeed) ? true : false;
}

int Choose_Next_Node(int node, const vector<int>& visitedNode, int& minRent, int& band){
    minRent = INT_MAX;
    int minNode = -1;
    EdgeNode *p = graph.adjList[node].outEdgeTails;
    while(p){
            //未访问过 && 该结点不是单边结点 && 路径租金小
        if((visitedNode[p -> tailID] == 1) && (!Only_One_Edge(p -> tailID)) && (p ->Rental < minRent) ){
            minNode = p -> tailID;
            minRent = p -> Rental;
            if(p -> bandWidth < band){
                band = p -> bandWidth;
            }
        }
        p = p -> vexOut;
    }
    return minNode;
}

void Init_Global_Path(vector<int>& cost, vector<int>& band, const vector<int>& antLife){
    int sumCost;
    vector<int> visitedNode;
    visitedNode.resize(graph.nodeNum);
    int currentNode;
    int nextNode;
    int minRent;
    int length;
    for(int i = 0; i < graph.custNum; ++i){
        //第i类蚂蚁
        sumCost = 0;
        length = 0;
        band[i] = INT_MAX;
        for(int i = 0; i < graph.nodeNum; ++i){
            visitedNode[i] = 1;
        }
        currentNode = graph.customerInfo[i].toNodeID;
        #if lim_debug
        cout << "Begin:" << currentNode << " ";
        #endif // lim_debug
        visitedNode[currentNode] = 0;
        for(int j = 0; j < antLife[i]; ++j){
            nextNode = Choose_Next_Node(currentNode, visitedNode, minRent, band[i]);
            if(nextNode >= 0){  //有可用下条结点
                sumCost += minRent;
                currentNode = nextNode;
                ++length;
                #if lim_debug
                cout << " (" << minRent << ") " << nextNode << " ";
                #endif // lim_debug
                visitedNode[currentNode] = 0;
            }
            else{
                break;  //无可用结点，蚂蚁进入死胡同，死蚂蚁，直接跳出循环
            }
        }
        cost[i] = sumCost;
        #if lim_debug
        cout << endl << "Length: " << length << endl;
        #endif // lim_debug
    }
}

/*蚁群类成员函数定义*/
void AntColonySystem::Estimate_Graph(){
    evaluate.server_rent_rate = graph.serverCost / graph.avgRent;
    evaluate.node_cust_rate = graph.nodeNum / graph.custNum;
}

void AntColonySystem::Init_Parameter(){
    Estimate_Graph();
    //初始化边上的信息素浓度
    for(int i = 0; i < graph.nodeNum; ++i){
        EdgeNode *p = graph.adjList[i].outEdgeTails;
        while(p){
            p -> edgePeromone = initParam.phe_max;  //最大最小蚁群
            p = p -> vexOut;
        }
    }
    //初始化结点上的信息素浓度, 结点信息素只积累不随时间减少？
    nodePheromone.resize(graph.nodeNum);
    for(int i = 0; i < graph.nodeNum; ++i){
        nodePheromone[i] = 0.0; //各结点初始信息素浓度全为0
    }
    //其他

//    deadAnt.resize(graph.nodeNum);
//    for(int i = 0; i < graph.nodeNum; ++i){
//        deadAnt[i] = 0;
//    }
}

double AntColonySystem::Transition_Chance(int currentNode, int nextNode){
    //查找该条边
    EdgeNode *p = graph.adjList[currentNode].outEdgeTails;
    while(p -> tailID != nextNode){
        p = p ->vexOut;
    }
    double pheromone = p ->edgePeromone;
    double rent = p -> Rental;

    if(currentNode != nextNode){
        return (pow(pheromone, initParam.alpha) * pow(rent, initParam.beta));
    }
    else
        return 0.0;
}

//局部更新规则
void AntColonySystem::Update_Local_Path_Rules(int currentNode, int nextNode, int antLife, int Lnn){
    //查找该条边
    EdgeNode *p = graph.adjList[currentNode].outEdgeTails;
    while(p -> tailID != nextNode){
        p = p ->vexOut;
    }
    p -> edgePeromone = (1 - initParam.rou_local) * p -> edgePeromone + initParam.rou_local * (1.0 / (antLife * Lnn));
}

//全局更新规则
void AntColonySystem::Update_Global_Rules(const vector<int>& global_minPath, int global_minCost){
    for(unsigned int i = 0; i < global_minPath.size() - 1; ++i){
        EdgeNode *p = graph.adjList[global_minPath[i]].outEdgeTails;
        while(p -> tailID != global_minPath[i + 1]){
            p = p ->vexOut;
        }
        p -> edgePeromone = (1 - initParam.rou) * p -> edgePeromone + initParam.rou * (1.0 / global_minCost);
    }
}

/*蚂蚁类成员函数定义*/
Ant::Ant(AntColonySystem *acs, int start, int life, Excite excite){
    antColony = acs;
    startNode = graph.customerInfo[start].toNodeID;
    antLife = life;
    excited = excite;
    currentBand_min = INT_MAX;
    aimNeed = graph.customerInfo[startNode].bandNeed;
    visitedNode.resize(graph.nodeNum);
    for(int i = 0; i < graph.nodeNum; ++i){
        visitedNode[i] = 1;
    }
}

void Ant::Search(int Lnn){
    currentNode = startNode;
    int nextNode;
    currentCost = 0;
    currentBand_min = INT_MAX;
    visitedNode[currentNode] = 0;
    route.push_back(currentNode);
    //走 antLife 步
    for(int i = 0; i < antLife; ++i){
        nextNode = Choose_Next(currentBand_min);
        if(nextNode >= 0){
            Move_to_Next_Node(nextNode);
            antColony ->Update_Local_Path_Rules(currentNode, nextNode, antLife, Lnn);
        }
        else{
            //antColony -> deadAnt += antLife - i;
            break;
        }
    }
    //return route;
}

int Ant::Choose_Next(int& minBand){
    int nextNode = -1;
    double q = rand() / (double)RAND_MAX;
    if (q <= initParam.q_zero){    //如果 q <= q0,按先验知识，从剩下节点中选择最大概率的可行节点,否则则按概率转移
        double probability = -1.0;  //转移到下一节点的概率
        EdgeNode *p = graph.adjList[currentNode].outEdgeTails;
        while(p){
            if(visitedNode[p -> headID] == 1){
                double prob = antColony ->Transition_Chance(p -> headID, p -> tailID);
                if(prob > probability){
                    nextNode = p -> tailID;
                    probability = prob;
                }
            }
            p = p -> vexOut;
        }
    }
    else{   //按概率转移
        double pr = rand()/(double)RAND_MAX; //生成一个随机数,用来判断落在哪个区间段
        double sum = 0.0;
        double probability = 0.0;           //概率的区间点，p 落在哪个区间段，则该点是转移的方向
        //计算概率公式的分母的值
        EdgeNode *p = graph.adjList[currentNode].outEdgeTails;
        while(p){
            if(visitedNode[p -> headID] == 1){
                sum += antColony ->Transition_Chance(p -> headID, p -> tailID);
            }
            p = p -> vexOut;
        }
        p = graph.adjList[currentNode].outEdgeTails;
        while(p){
            if(visitedNode[p -> headID] == 1){
                probability += antColony ->Transition_Chance(p -> headID, p -> tailID) / sum;
            }
            if (probability >= pr || ((pr > 0.9999) && (probability > 0.9999))){
                nextNode = p -> tailID;
                break;
            }
            p = p -> vexOut;
        }
    }
    return nextNode;
}

void Ant::Move_to_Next_Node(int nextNode){
    visitedNode[nextNode] = 0;
    route.push_back(nextNode);
    currentNode = nextNode;

}

/*蚁群算法*/
void AntColonyAlogrithm(const VideoNetGraph& Graph){
    Get_Graph(Graph);
    //初始化蚁群参数
    AntColonySystem *acs = new AntColonySystem();
    acs -> Init_Parameter();
    //初始化各蚁类信息
    vector<int> antLife;
    vector<Excite> excited;
    vector<int> minBand;
    vector<int> Lnns;   //Lnn数组
    antLife.resize(graph.custNum);
    excited.resize(graph.custNum);
    minBand.resize(graph.custNum);
    Lnns.resize(graph.custNum);
    for(int i = 0; i < graph.custNum; ++i){ //设置蚂蚁群参数，消费节点间距离越大，蚂蚁寿命越短；需求越高,寿命越短,亢奋度越高
        antLife[i] = graph.nodeNum / 2 - acs -> evaluate.node_cust_rate - graph.customerInfo[i].bandNeed / graph.avgBand;
        excited[i] = graph.customerInfo[i].bandNeed;
    }
    //初始化Lnns
    Init_Global_Path(Lnns, minBand, antLife);
    #if lim_debug
    for(int i = 0; i < graph.custNum; ++i){
        antLife[i] = graph.edgeNum;
    }
    for(int i = 0; i < graph.custNum; ++i){
        cout << graph.customerInfo[i].customerID << " best cost:" << Lnns[i] << endl;
    }
    #endif // lim_debug
    //初始化蚁类, 蚂蚁平均分布在各消费结点所连接的网络结点上
    int antNumber = graph.nodeNum;  //每类蚂蚁的个数
    Ant* ants[antNumber];

    for(int i = 0; i < graph.custNum; ++i){
        ants[i] = new Ant(acs, i, antLife[i], excited[i]);
    }
    //最优服务器位置，存全局最优路径，最优花费
    vector<int> serverPosition; // 服务器位置，个数范围：1-customNum,不需初始化
    vector<int> global_minCost;        //全局最小花费, custNum
    vector<vector<int> > global_minPath;    //全局最短路径,custNum
    global_minCost.resize(graph.custNum);
    global_minPath.resize(graph.custNum);

    //循环执行蚁群搜索
    for(int i = 0; i < initParam.loop_MAX; ++i){
        //局部最小解
        vector<int> local_minCost;
        vector<vector<int> > local_minPath;
        local_minCost.resize(graph.custNum);
        local_minPath.resize(graph.custNum);
        for(int l = 0; l < graph.custNum; ++l){
            local_minCost[l] = INT_MAX;
        }
        cout << "...1..." << endl;
        //单轮搜索
        for(int j = 0; j < antNumber; ++j){
            //各类蚂蚁各一次，交叉进行，增加信息交换
            cout << "custNum is " << graph.custNum << endl;
            for(int s = 0; s < graph.custNum; ++s){
                    cout << "...0..." << endl;
                ants[s] -> Search(Lnns[s]);
                if(ants[s] -> Get_CurrentCost() < local_minCost[ants[s] -> Get_StartNode()]){
                    local_minPath[ants[s] -> Get_StartNode()] = ants[s] -> Get_Route();
                }
            }
        }
        //本轮搜索最小花费比目前全局最小的还小
        for(int k = 0; k < graph.custNum; ++k){
            if(local_minCost[ants[k] -> Get_StartNode()] < global_minCost[ants[k] -> Get_StartNode()]){
                global_minCost[ants[k] -> Get_StartNode()] = local_minCost[ants[k] -> Get_StartNode()];
                global_minPath[ants[k] -> Get_StartNode()].resize(local_minPath[ants[k] -> Get_StartNode()].size());
                global_minPath[ants[k] -> Get_StartNode()] = local_minPath[ants[k] -> Get_StartNode()];

                acs -> Update_Global_Rules(global_minPath[ants[k] -> Get_StartNode()], global_minCost[ants[k] -> Get_StartNode()]);
            }

        }
        //输出第i次迭代后最优路径组
        #if lim_debug
        for(int m = 0; m < graph.custNum; ++m){
            cout << "The " << i << "st loop's minCost is: " << local_minCost[m] << endl;
        }
        #endif // lim_debug
    }
    //输出全局最优路径组
    #if lim_debug
    for(int i = 0; i < graph.custNum; ++i){
        cout << "...cust." << i <<"...minCost is: " << global_minCost[i] << endl;
    }
    #endif // lim_debug

    //释放空间
    delete[] ants;
    ///coding

}

//调试入口
void Debug_Into(){
    vector<int> Lnn;
    vector<int> minBand;
    Lnn.resize(graph.custNum);
    minBand.resize(graph.custNum);
    vector<int> antLife;
    antLife.resize(graph.custNum);
    for(int i = 0; i < graph.custNum; ++i){
        antLife[i] = graph.edgeNum;
        minBand[i] = INT_MAX;
    }
    cout << "antLife is " << antLife[0] << endl;
    Init_Global_Path(Lnn, minBand, antLife);
    for(int i = 0; i < graph.custNum; ++i){
        cout << graph.customerInfo[i].customerID << "'s begin Lnn:" << Lnn[i] << endl;
        cout << "... minBand is " << minBand[i] << endl;
    }
}

///直连输出
char* Direct_Connect(){
    stringstream inStream;
	string str_output = "", str_temp = "";
	inStream << graph.custNum;
	inStream >> str_output;
	inStream.clear();
	str_output = str_output + "\n\n";
	for(int i = 0; i < graph.custNum; ++i){
        inStream << graph.customerInfo[i].toNodeID;
        inStream >> str_temp;
        inStream.clear();
        str_output = str_output + str_temp + " ";

        inStream << graph.customerInfo[i].customerID;
        inStream >> str_temp;
        inStream.clear();
        str_output = str_output + str_temp + " ";

        inStream << graph.customerInfo[i].bandNeed;
        inStream >> str_temp;
        inStream.clear();
        str_output = str_output + str_temp + "\n";
	}
	return (char*) str_output.data();
}

void Get_Graph(const VideoNetGraph& Graph){
    graph = Graph;
}
