/*********************************************************************************************
Copyright: Limorton
Author: Limorton
Date: 2017-03-20-16.27
Description: 实现以下功能
    1、dijkstra算法求单源最短路径
    2、用Tarjan算法求图的割边
*********************************************************************************************/
#ifndef LIMORTON_DIJKSTRA_H_INCLUDED
#define LIMORTON_DIJKSTRA_H_INCLUDED
#include "limorton_Graph.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <queue>
#include <algorithm>
using namespace std;

static vector<int> notServers;
static vector<int> beServers;
//求割边参数
static vector<pair<int, int> > cutEdges;
static vector<int> num;
static vector<int> low;
int index = 0;
vector<int> flags;  //求割集着色

//记录最短路径及代价，最后以数组保存
struct min_path{
    int rent;
    int band;
    //vector<int> route;  //保存途径网络节点编号
};
//Dijkstra参数
static vector<min_path> minPath;
static vector<int> inSetS;
static vector<vector<min_path> > Min_Sum_Rent;  //距离数组

///比较两条链路租金，找较小的
struct cmp2node{
    bool operator()(const VertexNode& A, const VertexNode& B){
        return A.sum_rent > B.sum_rent;   //最小值优先
        //return A.sum_rent < B.sum_rent;   //最大值优先
    }
};

/** \brief
 *  比较两个数大小
 * \param a int
 * \param b int
 * \return int
 *
 */
int minAB(int a, int b){
    return a < b ? a : b;
}
/** \brief
 *  用Tarjan算法求图的割边
 * \param cur int--边的头节点
 * \param father int--边的尾结点
 * \return void
 *
 */
void Dfs_CutEdge(int cur, int father){
    index++;
    num[cur] = index;   //当前顶点的时间戳
    low[cur] = index;   //当前顶点能够访问到最早顶点的时间戳
    EdgeNode *p = graph.adjList[cur].outEdgeTails;
    while(p){
        if(num[p ->tailID] == 0){
            Dfs_CutEdge(p ->tailID, cur);
            low[cur] = minAB(low[cur], low[p ->tailID]);
            if(low[p ->tailID] > num[cur]){
                pair<int, int> cut(cur, p ->tailID);
                cutEdges.push_back(cut);
                cout << cur << "--" << p ->tailID << " is CutEdge" << endl;
            }
        }
        else if(p ->tailID != father){
            low[cur] = minAB(low[cur], num[p ->tailID]);
        }
        p = p ->vexOut;
    }
}

/** \brief
 *  初始化全局变量
 * \return void
 *
 */
void Init_Global_Variable(){
    min_path initP = {MY_INT_MAX, MY_INT_MAX};
    minPath.resize(graph.nodeNum, initP);
    inSetS.resize(graph.nodeNum, 1);
    Min_Sum_Rent.resize(graph.nodeNum);
    for(int i = 0; i < graph.nodeNum; ++i){
        Min_Sum_Rent[i].resize(graph.custNum);
        graph.adjList[i].setServer = false;
    }
    for(int i = 0; i < graph.custNum; ++i){
        graph.customerInfo[i].inPairs = false;
    }
}

/** \brief
 *  恢复变量
 * \return void
 *
 */
void Reset_Variable(){
    for(int i = 0; i < graph.nodeNum; ++i){
        minPath[i].rent = MY_INT_MAX;
        minPath[i].band = MY_INT_MAX;
        graph.adjList[i].sum_rent = MY_INT_MAX;
        inSetS[i] = 1;
    }
}

/** \brief
 *  Dijkstra算法求各网络结点到消费结点的最低租金路径
 * \param start const Customer&--消费结点编号
 * \return void
 *
 */
void Dijkstra(const Customer& start){
    priority_queue<VertexNode, vector<VertexNode>, cmp2node> S;
    VertexNode u;
    S.push(graph.adjList[start.toNodeID]);
    minPath[start.toNodeID].rent = 0;
    minPath[start.toNodeID].band = MY_INT_MAX;
    //minPath[start.toNodeID].route.push_back(start.toNodeID);
    inSetS[start.toNodeID] = 0;
    while(!S.empty()){
        u = S.top();
        S.pop();
        inSetS[u.nodeID] = 0;
        EdgeNode *p = graph.adjList[u.nodeID].outEdgeTails;
        while(p){   //查找相邻顶点，更新信息并压入S
            if(minPath[p ->tailID].rent > minPath[u.nodeID].rent + p ->Rental){
                minPath[p ->tailID].rent = minPath[u.nodeID].rent + p ->Rental;
                graph.adjList[p ->tailID].sum_rent = graph.adjList[u.nodeID].sum_rent + p ->Rental;
                //minPath[p ->tailID].route = minPath[u.nodeID].route;
                //minPath[p ->tailID].route.push_back(p ->tailID);
                minPath[p ->tailID].band = (minPath[u.nodeID].band < p ->bandWidth) ? (minPath[u.nodeID].band) : p ->bandWidth;
                if(inSetS[p ->tailID] == 1){
                    inSetS[p ->tailID] = 0;
                    S.push(graph.adjList[p ->tailID]);
                }
            }
            p = p ->vexOut;
        }
    }
    for(int i = 0; i < graph.nodeNum; ++ i){
        Min_Sum_Rent[i][start.customerID] = minPath[i];
    }
    #if 0
    cout << "from Node " << start.toNodeID << endl;
    for(int i = 0; i < graph.nodeNum; ++i){
        cout << "Node " << i << ", rent: " << minPath[i].rent << ", band: " << minPath[i].band << endl;
        cout << "Path: ";
        for(unsigned int j = 0; j < minPath[i].route.size(); ++j){
            cout << minPath[i].route[j] << " -> ";
        }
        cout << endl;
    }
    #endif // 0
}

/** \brief
 *  运行Dijkstra算法
 * \param beCustSer const vector<int>&--必设服务器结点的网络结点
 * \return void
 *
 */
void Run_Dijkstra(const vector<int>& beCustSer){
    Init_Global_Variable();
    for(int i = 0; i < graph.custNum; ++i){
        if(beCustSer[i] == 0){
            Dijkstra(graph.customerInfo[i]);
            Reset_Variable();
        }
    }
    #if Lim_Debug && 0
    cout << "run_dijkstras:" << endl;
    for(int i = 0; i < graph.nodeNum; ++i){
        cout << i << ": ";
        for(int j = 0; j < graph.custNum; ++j){
            cout << Min_Sum_Rent[i][j].rent << " ";
        }
        cout << endl;
    }
    #endif // Lim_Debug
}

/** \brief
 *  判断nodeNum是否是消费结点连接的结点
 * \param nodeNum int--网络结点编号
 * \return bool--不是返回true 是返回false
 *
 */
bool NotCusttoNode(int nodeNum){
    for(int i = 0; i < graph.custNum; ++i)
        if(nodeNum == graph.customerInfo[i].toNodeID)
            return false;
    return true;
}

/** \brief
 *  求邻近的已设置服务器的结点的数量
 * \param nodeNum int--网络结点编号
 * \return int--邻近的已设置服务器的结点的数量
 *
 */
int CountNearServer(int nodeNum){
    int servNum = 0;
    EdgeNode *p = graph.adjList[nodeNum].outEdgeTails;
    while(p){
        if(graph.adjList[p ->tailID].setServer == true)
            ++servNum;
        p = p ->vexOut;
    }
    return servNum;
}

/** \brief
 *  由网络节点求其连接的消费结点
 * \param nodeNum int--网络结点编号
 * \return int--对应消费结点编号
 *
 */
int Cust_Connect(int nodeNum){
    for(int i = 0; i < graph.custNum; ++i)
        if(nodeNum == graph.customerInfo[i].toNodeID)
            return i;
    return -1;
}

/** \brief
 *  启发式：当一个网络结点仅有一条出边，且无消费结点连接时,一定不会是服务器结点
 * \param nodeNum int--网络结点编号
 * \return bool--必不是服务器结点返回true
 *
 */
bool Only_One_Edge(int nodeNum){
    return (((graph.adjList[nodeNum].edgeAttached == 1) && \
             NotCusttoNode(nodeNum))? true : false);
}

/** \brief
 *  启发式：消费节点连接的网络节点的所有路径的带宽总和小于消费节点需求带宽，必是服务器结点
 * \param customNum int--消费结点编号
 * \return bool--必是服务器结点返回true
 *  极端情况：Net -> lonelyNode -> ... -> lonelyNode -> custNode 且该链路最小带宽小于消费结点需求
 */
bool Flux_Over_Flow(int customNum){
    return (graph.adjList[graph.customerInfo[customNum].toNodeID].maxFlux \
            < graph.customerInfo[customNum].bandNeed) ? true : false;
}

/** \brief
 *  启发式：需求不超带宽的前提下，可能的最小流量花销大于单个服务器安装费用
 * \param custNum int--消费结点编号
 * \return bool--必是服务器结点返回true
 *
 */
bool Cost_Bigger_Node(int custNum){
    //贪心计算可能的最小流量花费
    EdgeNode *p;
    int sumBand = 0, sumCost = 0;
    int minRent = MY_INT_MAX;
    int band = 0;
    int caldown = 0;
    while(sumBand < graph.customerInfo[custNum].bandNeed){
        p = graph.adjList[graph.customerInfo[custNum].toNodeID].outEdgeTails;
        while(p && (p -> tailID < graph.nodeNum)){
            //if(p -> vexOut -> vexOut)
            if(p ->Rental < minRent && p ->rest_bandWidth != 0){
                band = p ->bandWidth;
                minRent = p ->Rental;
            }
            p = p ->vexOut;
        }
        p = graph.adjList[graph.customerInfo[custNum].toNodeID].outEdgeTails;
        while(p && (p -> tailID < graph.nodeNum)){
            if(p ->Rental == minRent && p ->bandWidth == band && p ->rest_bandWidth != 0){
                if(sumBand + band < graph.customerInfo[custNum].bandNeed){
                    p ->rest_bandWidth = 0;
                    sumBand += band;
                    sumCost += minRent * band;
                    minRent = MY_INT_MAX;
                    break;
                }
                else{
                    sumCost += (graph.customerInfo[custNum].bandNeed - sumBand) * p ->Rental;
                    caldown = 1;
                    break;
                }

            }
            p = p ->vexOut;
        }
        if(caldown)
            break;
    }
    //恢复边信息
    p = graph.adjList[graph.customerInfo[custNum].toNodeID].outEdgeTails;
    while(p){
        p ->rest_bandWidth = p ->bandWidth;
        p = p ->vexOut;
    }
    return (sumCost >= graph.serverCost) ? true : false;
}

/** \brief
 *  为结点着色
 * \param nodeNum int--网络结点编号
 * \return void
 *  用于求割边
 */
void Paint_Node(int nodeNum){
    EdgeNode *p = graph.adjList[nodeNum].outEdgeTails;
    while(p){
        if(flags[p ->tailID] == 0){ //未着色
            flags[p ->tailID] = flags[nodeNum];
            Paint_Node(p ->tailID);
        }
        p = p ->vexOut;
    }
}

/** \brief
 *  判断color的结点集合是否有消费结点
 * \param color int--颜色标识
 * \return bool--有消费结点返回true
 *
 */
bool hasCust(int color){
    bool hasCust = false;
    for(int i = 0; i < graph.nodeNum; ++i){
        if(flags[i] == color && Cust_Connect(i) != -1){
            hasCust = true;
            break;
        }
    }
    return hasCust;
}

/** \brief
 *  删除孤立结点连接的边，并返回孤立结点集合
 * \return vector<int>--孤立结点集合
 *
 */
vector<int> Find_Lonely_Node(){
    vector<int> lonlyNodes;
    int stillHave = 1;
    while(stillHave){
        stillHave = 0;
        for(int i = 0; i < graph.nodeNum; ++i){
            if(Only_One_Edge(i)){
                EdgeNode *p = graph.adjList[i].outEdgeTails;
                int j = p -> tailID;
                Delete_OneEdge(i, j);
                Delete_OneEdge(j, i);
                graph.adjList[i].edgeAttached = 0;
                --graph.adjList[j].edgeAttached;
                lonlyNodes.push_back(i);
                //cout << "cut " << i << endl;
                stillHave = 1;
            }
        }
    }
    //求图的不含消费结点的割集, 实际中未出现此情况，不使用此方法
//    num.resize(graph.nodeNum, 0);
//    low.resize(graph.nodeNum, 0);
//    Dfs_CutEdge(0, 0);
//    flags.resize(graph.nodeNum, 0);
//    for(u_int i = 0; i < cutEdges.size(); ++i){
//        flags.assign(graph.nodeNum, 0);
//        bool canCut = false;
//        flags[cutEdges[i].first] = 1;
//        flags[cutEdges[i].second] = 2;
//        int band = Get_Band(cutEdges[i].first, cutEdges[i].second);
//        int rent = Get_Rent(cutEdges[i].first, cutEdges[i].second);
//        Delete_OneEdge(cutEdges[i].first, cutEdges[i].second);
//        Delete_OneEdge(cutEdges[i].second, cutEdges[i].first);
//        flags[cutEdges[i].first] = 1;
//        Paint_Node(cutEdges[i].first);
//        flags[cutEdges[i].second] = 2;
//        Paint_Node(cutEdges[i].second);
//        if(!hasCust(1)){    //1不含消费结点
//            for(int j = 0; j < graph.nodeNum; ++j){
//                if(flags[j] == 1){
//                    lonlyNodes.push_back(j);
//                    cout << "cut " << j << endl;
//                }
//            }
//            canCut = true;
//        }
//        if(!hasCust(2)){    //2不含消费结点
//            for(int j = 0; j < graph.nodeNum; ++j){
//                if(flags[j] == 2){
//                    lonlyNodes.push_back(j);
//                    cout << "cut " << j << endl;
//                }
//            }
//            canCut = true;
//
//        }
//        if(canCut)
//            cout << "can Cut " << cutEdges[i].first << "--" << cutEdges[i].second << endl;
//        if(!canCut){
//            Add_OneEdge(cutEdges[i].first, cutEdges[i].second, band, band, rent);
//            Add_OneEdge(cutEdges[i].second, cutEdges[i].first, band, band, rent);
//        }
//            Add_OneEdge(cutEdges[i].first, cutEdges[i].second, band, band, rent);
//            Add_OneEdge(cutEdges[i].second, cutEdges[i].first, band, band, rent);
//    }
    return lonlyNodes;
}

/** \brief
 *  找到必就近设服务器的消费结点，并返回对应连接结点编号集合
 * \return vector<int>--必就近设服务器的消费结点连接结点的编号集合
 *
 */
vector<int> Find_Be_Node(){
    vector<int> mustBeID;
    for(int i = 0; i < graph.custNum; ++i){
        if(Flux_Over_Flow(i)){
            mustBeID.push_back(graph.customerInfo[i].toNodeID);
        }
        else if(Cost_Bigger_Node(i)){
            mustBeID.push_back(graph.customerInfo[i].toNodeID);
        }
    }
    return mustBeID;
}

/** \brief
 *  找到必就近设服务器的消费结点，并返回消费结点编号集合
 * \param beServers const vector<int>&--必就近设服务器的消费结点连接结点的编号集合
 * \return vector<int>--必就近设服务器的消费结点的编号集合
 *
 */
vector<int> Find_Be_Cust(const vector<int>& beServers){
    vector<int> mustBeID(graph.custNum, 0);
    int beSize = beServers.size();
    for(int i = 0; i < beSize; ++i){
        for(int j = 0; j < graph.custNum; ++j){
            if(graph.customerInfo[j].toNodeID == beServers[i]){
                mustBeID[j] = 1;
                break;
            }
        }
    }
    return mustBeID;
}


#endif // LIMORTON_DIJKSTRA_H_INCLUDED
