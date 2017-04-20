/*********************************************************************************************
Copyright: Limorton
Author: Limorton
Date: 2017-03-23-16.02
Description: 实现最小费用最大流算法，采用贪心的思想，每次找到一条从源点到达汇点的路径，增加流量，
  且该条路径满足使得增加的流量的花费最小，直到无法找到一条从源点到达汇点的路径，算法结束。由于
  最大流量有限，每执行一次循环流量都会增加，因此该算法肯定会结束，且同时流量也必定会达到网络的
  最大流量；同时由于每次都是增加的最小的花费，即当前的最小花费是所有到达当前流量flow时的花费最
  小值，因此最后的总花费最小。
  求解步骤
    （1）找到一条从源点到达汇点的“距离最短”的路径，“距离”使用该路径上的边的单位费用之和来衡量。 
    （2）然后找出这条路径上的边的容量的最小值f，则当前最大流max_flow扩充f，同时当前最小费用min_cost扩充 f*min_dist(s,t)。 
    （3）将这条路径上的每条正向边的容量都减少f，每条反向边的容量都增加f。 
    （4）重复（1）--（3）直到无法找到从源点到达汇点的路径。
  需要注意几点： 
    1、注意超级源点和超级终点的建立。 
    2、初始化时，正向边的单位流量费用为cost[u][v]，那么反向边的单位流量费用就为-cost[u][v]。因为回流费用减少。 
    3、费用cost数组和容量cap数组每次都要初始化为0。
       求解从源点到汇点的“最短”路径时，由于网络中存在负权边，因此使用SPFA来实现。
**********************************************************************************************/
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
/** \brief
 *  计算部署的服务器数量
 * \return int--服务器数量
 *
 */
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
/** \brief
 *  输出显示服务器路线
 * \return void
 *
 */
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

/** \brief
 *  输出当前最短路径链路
 * \return void
 *
 */
void Display_Route(){
    cout << ">>>> Routes(Num= " << ANS_ROUTE.size() <<", Server= " << Cal_ServersNum() << "):" << endl;
    for(unsigned int i = 0; i < ANS_ROUTE.size(); ++i){
        copy(ANS_ROUTE[i].begin(), ANS_ROUTE[i].end(), ostream_iterator<int>(cout, " "));
        cout << endl;
    }
    cout << ">>>>" << endl;
}

/** \brief
 *  获取网络结点连接的消费节点
 * \param toNode int--网络结点编号
 * \return int--消费节点编号
 *
 */
int Get_CustID(int toNode){
    for(int i = 0; i < graph.custNum; ++i){
        if(graph.customerInfo[i].toNodeID == toNode)
            return i;
    }
    return -1;
}

/** \brief
 *  储存运行路径
 * \param start int--出发结点
 * \param endnode int--目的结点
 * \return void
 *
 */
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

/** \brief
 *  最小费用最大流算法前初始化全局变量
 * \return void
 *  每次运行前都要初始化
 */
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

/** \brief
 *  SPFA算法
 * \param startNode int--出发结点
 * \param endNode int--目的结点
 * \return bool--true:成功找到增广路
 *  返回false表示无增广路
 */
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

/** \brief
 *  检测是否将服务器设置在不在不能设服务器结点的结点
 * \param notServers const vector<int>&--不能设服务器结点的结点
 * \param node int--待检测的服务器结点
 * \return bool--不在其中返回true
 *
 */
bool NotInNot(const vector<int>& notServers, int node){
    for(u_int i = 0; i < notServers.size(); ++i){
        if(notServers[i] == node)
            return false;
    }
    return true;
}

#if Lim_Debug
/** \brief
 *  检查边流量是否超带宽
 * \return void
 *
 */
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
/** \brief
 *  检查是否有服务器设置错误
 * \param ServerNode const vector<int>&--当前服务器结点
 * \param notServers const vector<int>&--不能设服务器的结点
 * \param beServers const vector<int>&--必设服务器的结点
 * \return void
 *
 */
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

/** \brief
 *  检查路径是否错误
 * \param lastTime bool--是否是最后一次运行，调试用的参数
 * \return bool--无效解返回true:放弃本次更新最优解
 *  检查包括:
        1、检查是否满足各消费结点需求
        2、检查是否有流量超带宽
 */
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

/** \brief
 *  最小费用最大流(MCMF)算法
 * \param startNode int--出发结点
 * \param endNode int--目的结点
 * \return int--最小费用值(目的函数)
 *  实际使用中，出发结点为超级源结点，目的结点为超级终结点
 */
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

/** \brief
 *  添加超级节点后运行MCMF算法
 * \param ServerNode const vector<int>&--设置服务器的结点集合
 * \param lastTime bool--是否是最后一次运行，调试用参数
 * \return int--最小费用值(目的函数)
 *
 */
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

