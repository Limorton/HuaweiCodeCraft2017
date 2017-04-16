#ifndef LIMORTON_MCMF_H_INCLUDED
#define LIMORTON_MCMF_H_INCLUDED
#include <cstdio>
#include <iostream>
#include <cmath>
#include <iterator>
#include <queue>
#include <algorithm>
#include "limorton_dijkstra.h"
using namespace std;

static int oneFlow = FLOW_MAX;
static vector<int> distances;   //存储到初始结点的距离，nodeNum + 2
//static vector<int> countForQueue; //记录各个结点的入队次数，若超过nodeNum + 2则说明出现负环，退出
static deque<VertexNode> Releax;   //队列，记录需要松弛的结点
static vector<bool> inQueue;   //用于判断结点是否已经在队列中
static vector<int> preNode;     //记录当前结点的前一个结点

static vector<vector<int> > ANS_ROUTE;  //一次MCMF的最优链路集
static vector<int> ans_OneInRoute;

///计算部署的服务器数量
int Cal_ServersNum(){
    int routeNum = ANS_ROUTE.size();
    vector<int> calNum(routeNum);
    for(int i = 0; i < routeNum; ++i){
        calNum[i] = ANS_ROUTE[i][0];
    }
    sort(calNum.begin(), calNum.end());
    int pre = calNum[0];
    int cur;
    int serverNum = 1;
    for(int i = 1; i < routeNum; ++i){
        cur = calNum[i];
        if(pre != cur){
            ++serverNum;
            pre = cur;
        }
    }
    return serverNum;
}

#if Lim_Debug
void Disp_RouteServer(){
    cout << "#RServer At:" << endl;
    int routeNum = ANS_ROUTE.size();
    vector<int> calNum(routeNum);
    for(int i = 0; i < routeNum; ++i){
        calNum[i] = ANS_ROUTE[i][0];
    }
    sort(calNum.begin(), calNum.end());
    int pre = calNum[0];
    int cur;
    cout << "_" << pre;
    for(int i = 1; i < routeNum; ++i){
        cur = calNum[i];
        if(pre != cur){
            cout << "_" << cur;
            pre = cur;
        }
    }
    cout << endl;
}
#endif // Lim_Debug


///输出某点的最短路径链路
void Display_Route(){
    cout << ">>>> Routes(Num= " << ANS_ROUTE.size() <<", Server= " << Cal_ServersNum() << "):" << endl;
    for(unsigned int i = 0; i < ANS_ROUTE.size(); ++i){
        copy(ANS_ROUTE[i].begin(), ANS_ROUTE[i].end(), ostream_iterator<int>(cout, " "));
        cout << endl;
    }
    cout << ">>>>" << endl;
}

///获取网络结点连接的消费节点
int Get_CustID(int toNode){
    for(int i = 0; i < graph.custNum; ++i){
        if(graph.customerInfo[i].toNodeID == toNode)
            return i;
    }
    return -1;
}

///储存路径
void Update_Route(int start, int endnode){
    ans_OneInRoute.clear();
    //ans_OneInRoute.push_back(endnode); //有超级源
    int i = preNode[endnode];
    while(i != -1){
        ans_OneInRoute.push_back(i);
        i = preNode[i];
    }
    if(ans_OneInRoute.size() > 1){
        ans_OneInRoute.pop_back();    //无超级汇点
        reverse(ans_OneInRoute.begin(), ans_OneInRoute.end());
        //插入消费结点和占用带宽大小
        ans_OneInRoute.push_back(Get_CustID(ans_OneInRoute.at(ans_OneInRoute.size() - 1)));
        ans_OneInRoute.push_back(oneFlow);
        ANS_ROUTE.push_back(ans_OneInRoute);
    }
}

///费用每次都要初始化
void Init_Variables(){
    ANS_ROUTE.clear();
    ans_OneInRoute.clear();
    Releax.clear();
    oneFlow = FLOW_MAX;
    distances.clear();
    distances.resize(graph.nodeNum + 2);
    //countForQueue.resize(graph.nodeNum +2);
    inQueue.clear();
    inQueue.resize(graph.nodeNum + 2);
    preNode.clear();
    preNode.resize(graph.nodeNum + 2);
    //重设每条边
    //Print_Graph();
    for(int i = 0; i < graph.nodeNum + 2; ++i){ //包含超级结点
        EdgeNode *p = graph.adjList[i].outEdgeTails;
        while(p){
            p ->rest_bandWidth = p ->bandWidth;
            p = p -> vexOut;
        }
    }
}

///SPFA，返回0表示无增广路
bool SPFA(int startNode, int endNode){
    for(int i = 0; i < graph.nodeNum + 2; ++i){
            distances[i] = DIST_MAX;
            inQueue[i] = false;
            preNode[i] = -1;
    }
    distances[startNode] = 0;
    inQueue[startNode] = true;
    Releax.push_front(graph.adjList[startNode]);
    VertexNode u;
    while(!Releax.empty()){
        u = Releax.front();
        Releax.pop_front();
        inQueue[u.nodeID] = false;
        EdgeNode *p = graph.adjList[u.nodeID].outEdgeTails;
        while(p){
            if(p ->rest_bandWidth > 0 && distances[p ->tailID] > distances[p ->headID] + p ->Rental){
                distances[p ->tailID] = distances[p ->headID] + p ->Rental;
                preNode[p ->tailID] = p ->headID;

                if(!inQueue[p ->tailID]){
                    inQueue[p ->tailID] = true;
                    if(!Releax.empty()){    //队列优化SLF策略，该点距离小于队首点的距离则插入队首，否则插入队尾
                        if(distances[p ->tailID] < distances[Releax.front().nodeID])
                            Releax.push_front(graph.adjList[p ->tailID]);
                        else
                            Releax.push_back(graph.adjList[p ->tailID]);
                    }else{
                        Releax.push_back(graph.adjList[p ->tailID]);
                        //cout << "push in: " << p ->tailID << endl;
                    }
                }
            }
            p = p -> vexOut;
        }
    }
    if(distances[endNode] == DIST_MAX)   //说明不存在到达endnode 的路径
        return false;
    return true;
}

bool NotInNot(const vector<int>& notServers, int node){
    for(u_int i = 0; i < notServers.size(); ++i){
        if(notServers[i] == node)
            return false;
    }
    return true;
}

#if Lim_Debug
void Check_Bands(){
    //重新初始化后查边租金是否正常
    for(int no = 0; no < graph.nodeNum; ++no){
        EdgeNode *p = graph.adjList[no].outEdgeTails;
        while(p){
            if(p ->rest_bandWidth < p ->bandWidth){
                cout << "Band Error__" << p ->headID << "->" << p ->tailID << "band: " << p ->bandWidth << ", rest_band:" << p ->rest_bandWidth << endl;;
            }
            p = p ->vexOut;
        }
    }
}
#endif // Lim_Debug

#if Lim_Debug
///检查状态是否有数据错误
void CheckOK(const vector<int>& ServerNode, const vector<int>& notServers, const vector<int>& beServers){

    //检查服务器结点数量
    int counter = 0;
    for(int i = 0; i < graph.nodeNum; ++i){
        if(ServerNode[i] > 0)
            ++counter;
    }
    if(counter != graph.guessSerNum){
        cout << "####Server Num Error__is" << counter << endl;
        ++serverNum_err;
    }

    #if 0
        //检查是否有违反启发式
        for(int i = 0; i < graph.nodeNum; ++i){
            if((ServerNode[i] == 1 || ServerNode[i] == 2) && !NotInNot(notServers, i)){
                cout << "#####Should Not be at: " << i << endl;
            }
        }
    #endif // 0

    for(u_int i = 0; i < beServers.size(); ++i){
        if(ServerNode[beServers[i]] <= 0){
            cout << "####Should be, but Not be, at: " << beServers[i] << endl;
        }
    }
}
#endif // Lim_Debug

///true: 无效解,放弃本次更新最优解
bool CheckRoute(bool lastTime){
    //检查是否满足各消费结点需求
    int disp = 0;
    for(int i = 0; i < graph.custNum; ++i){
        if(Check_Edge(graph.customerInfo[i].toNodeID, graph.nodeNum + 1) < graph.customerInfo[i].bandNeed){
            #if Lim_Debug
                if(lastTime)
                    cout << "####Not Need at cust " << i << ", needs " << graph.customerInfo[i].bandNeed << endl;
                ++routeNeed_err;
            #endif // Lim_Debug
            disp = 1;
        }
    }

    #if Lim_Debug
        //检查是否有流量超带宽
        for(u_int i = 0; i < ANS_ROUTE.size(); ++i){
            if(ANS_ROUTE[i].size() <= 3)
                continue;
            for(u_int j = 0; j < ANS_ROUTE[i].size() - 3; ++j){
                if(Check_Band(ANS_ROUTE[i][j], ANS_ROUTE[i][j + 1]) < 0){
                    #if Lim_Debug
                        cout << "####Band Overflow at " << "(" << ANS_ROUTE[i][j] << " , " << ANS_ROUTE[i][j + 1] << ")" << endl;
                        ++edgeband_err;
                    #endif // Lim_Debug
                    disp = 1;
                }
            }
        }
    #endif // Lim_Debug
    if(disp == 1){
        //Display_Route();
        return true;    //需求不满足，无效解
    }
    return false;
}

///MCMF
int MinCostMaxFlow(int startNode, int endNode){
    Init_Variables();
    int cost = 0;
    int flows = 0;
    while(SPFA(startNode, endNode)){
        oneFlow = FLOW_MAX;
        //找到该次SPFA搜索到的最短路径上的最小带宽
        for(int v = endNode; v != startNode; v = preNode[v]){
            int u = preNode[v];
            EdgeNode *p = graph.adjList[u].outEdgeTails;
            while(p ->tailID != v){
                p = p ->vexOut;
            }
            if(p ->rest_bandWidth < oneFlow){
                oneFlow = p ->rest_bandWidth;
            }
        }
        flows += oneFlow;
        cost += distances[endNode] * oneFlow;
        //更新该路径上的残余带宽
        for(int v = endNode; v != startNode; v = preNode[v]){
            int u = preNode[v];
            EdgeNode *p = graph.adjList[u].outEdgeTails;
            //正向边容量减少
            while(p ->tailID != v){
                p = p ->vexOut;
            }
            p ->rest_bandWidth -= oneFlow;
            //反向边容量增加
            EdgeNode *q = graph.adjList[p ->tailID].outEdgeTails;
            while(q ->tailID != u){
                q = q ->vexOut;
            }
            q ->rest_bandWidth += oneFlow;
        }
        Update_Route(startNode, endNode);
    }
    return cost;
}

///添加超级节点后运行MCMF算法
int Run_MCMF(const vector<int>& ServerNode, bool lastTime){
    Add_SuperSourceNodeEdge(ServerNode);
    Init_Variables();
    //Check_Bands();
    int rentCost = MinCostMaxFlow(graph.nodeNum, graph.nodeNum + 1);
    int servNum = Cal_ServersNum();
    int totalCost = graph.serverCost * servNum + rentCost;
    #if Lim_Debug
        if(lastTime){
            cout << "### GlbBestFx  =  " << totalCost << "/" << graph.dirCost << " = " \
            << graph.serverCost << " * " << servNum << " + " << rentCost << " (N = " << ANS_ROUTE.size() << ")" << endl;
        }
    #endif // Lim_Debug
    Destory_SuperSourceNodeEdge(ServerNode);
    if(CheckRoute(lastTime))
        return MY_INT_MAX;
    return totalCost;
}

#endif // LIMORTON_MCMF_H_INCLUDED

