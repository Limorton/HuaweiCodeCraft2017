#include "limorton_Graph.h"
#include "lib_io.h"
#include "lib_time.h"
#include <sstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
using namespace std;

VideoNetGraph graph;

///输出显示结点i的出边
void Print_Edges(int i){
    cout << graph.adjList[i].nodeID << "--out--(" << graph.adjList[i].edgeAttached << "," << graph.adjList[i].maxFlux << ")" << "-->";
    EdgeNode *p = graph.adjList[i].outEdgeTails;
    while(p){
        cout << "(" << p ->headID << ", " << p -> tailID << ", " << p -> bandWidth << ", " << p -> rest_bandWidth<< ", " << p -> Rental << ")";
        p = p -> vexOut;
    }
    cout << endl;
    p = graph.adjList[i].inEdgeTails;
    cout << graph.adjList[i].nodeID << "--in---(" << graph.adjList[i].edgeAttached << "," << graph.adjList[i].maxFlux << ")" << "-->";
    while(p){
        cout << "(" << p ->headID << ", " << p -> tailID << ", " << p -> bandWidth << ", " << p -> rest_bandWidth << ", " << p -> Rental << ")";
        p = p -> vexIn;
    }
    cout << endl << endl;
}


///初始拓扑图
void Init_Graph(char** topo){
    //获取基本信息
    stringstream inStream;
    inStream << topo[0];
    inStream >> graph.nodeNum >> graph.edgeNum >> graph.custNum;
    inStream.clear();
    inStream << topo[2];
    inStream >> graph.serverCost;
    inStream.clear();
    graph.adjList.resize(graph.nodeNum + 2);    //包含2个超级结点
    graph.customerInfo.resize(graph.custNum);
    graph.dirCost = graph.serverCost * graph.custNum;
}

///添加一条边
void Add_OneEdge(int head, int tail, int band, int r_band, int rent){
    EdgeNode* pEdge = new EdgeNode;
    pEdge ->headID = head;
    pEdge ->tailID = tail;
    pEdge ->bandWidth = band;
    pEdge ->rest_bandWidth = r_band;
    pEdge ->Rental = rent;

    pEdge -> vexOut = graph.adjList[head].outEdgeTails;
    if(pEdge -> vexOut){
        while(pEdge -> vexOut -> vexOut){   //找到链表adjList[head].outEdgeTails的尾部
           pEdge -> vexOut = pEdge -> vexOut -> vexOut;
        }
        pEdge -> vexOut -> vexOut = pEdge;  //插入尾部
        pEdge -> vexOut = nullptr;
    }
    else{   //链表adjList[head].outEdgeTails为空
         graph.adjList[head].outEdgeTails = pEdge;
         pEdge -> vexOut = nullptr;
    }

    pEdge -> vexIn = graph.adjList[tail].inEdgeTails;
    if(pEdge -> vexIn){
        while(pEdge -> vexIn -> vexIn){ //找到链表adjList[head].inEdgeTails的尾部
            pEdge -> vexIn = pEdge -> vexIn -> vexIn;
        }
        pEdge -> vexIn -> vexIn = pEdge;
        pEdge -> vexIn = nullptr;
    }
    else{
        graph.adjList[tail].inEdgeTails = pEdge;
        pEdge -> vexIn = nullptr;
    }
}

///删除一条边
void Delete_OneEdge(int head, int tail){
    EdgeNode *p, *q;
    //修改head的出弧链表
    if(graph.adjList[head].outEdgeTails ->tailID == tail){  //在链表头
        q = graph.adjList[head].outEdgeTails;
        if(q ->vexOut){                                     //该边后有边
            graph.adjList[head].outEdgeTails = q ->vexOut;
        }
        else{                                               //该边后无边
            graph.adjList[head].outEdgeTails = nullptr;
        }
    }
    else{                                                   //不在链表头
        p = graph.adjList[head].outEdgeTails;
        while(p && (p ->vexOut) && (p ->vexOut ->tailID != tail)){
            p = p ->vexOut;
        }                                                   //在链表中寻找该边
        if(p && (p ->vexOut -> vexOut)){
            q = p ->vexOut;
            p -> vexOut = q -> vexOut;
        }
        else{   //该边无出边，即要删除的边在链表末尾
            p -> vexOut = nullptr;
        }
    }
    //修改tail的入弧链表
    if(graph.adjList[tail].inEdgeTails ->headID == head){
        q = graph.adjList[tail].inEdgeTails;
        if(q ->vexIn){
            graph.adjList[tail].inEdgeTails = q ->vexIn;
        }
        else{
            graph.adjList[tail].inEdgeTails = nullptr;
        }
        delete q;   //删除该边
    }
    else{
        p = graph.adjList[tail].inEdgeTails;
        while(p && (p ->vexIn) && (p ->vexIn ->headID != head)){
            p = p ->vexIn;
        }                                           //在链表中寻找该边
        if(p && (p -> vexIn -> vexIn)){ //删除该边
            q = p ->vexIn;
            p -> vexIn = q -> vexIn;
            delete q;
        }
        else{
            q = p ->vexIn;
            delete q;
            p -> vexIn = nullptr;
        }
    }
}

///添加超级源边
void Add_SuperSourceNodeEdge(const vector<int>& ServerNode){
    int servNum = ServerNode.size();
    for(int i = 0; i < servNum; ++i){
        if(ServerNode[i] == 1 || ServerNode[i] == 2){
            Add_OneEdge(graph.nodeNum, i, BAND_MAX, BAND_MAX, 0);
            Add_OneEdge(i, graph.nodeNum, BAND_MAX, BAND_MAX, 0);
        }
    }
}

///删除超级源边
void Destory_SuperSourceNodeEdge(const vector<int>& ServerNode){
    int servNum = ServerNode.size();
    for(int i = 0; i < servNum; ++i){
        if(ServerNode[i] == 1 || ServerNode[i] == 2){
            Delete_OneEdge(graph.nodeNum, i);
            Delete_OneEdge(i, graph.nodeNum);
        }
    }
}

///添加超级汇边
void Add_SuperEndNodeEdge(){
    for(int i = 0; i < graph.custNum; ++i){
        Add_OneEdge(graph.nodeNum + 1, graph.customerInfo[i].toNodeID, graph.customerInfo[i].bandNeed, graph.customerInfo[i].bandNeed, 0);
        Add_OneEdge(graph.customerInfo[i].toNodeID, graph.nodeNum + 1, graph.customerInfo[i].bandNeed, graph.customerInfo[i].bandNeed, 0);
    }
}

///删除超级汇边
void Destory_SuperEndNodeEdge(){
    for(int i = 0; i < graph.custNum; ++i){
        Delete_OneEdge(graph.nodeNum + 1, graph.customerInfo[i].toNodeID);
        Delete_OneEdge(graph.customerInfo[i].toNodeID, graph.nodeNum + 1);
    }
}

///生成拓扑图
void Create_Graph(char** topo){
    //初始化网络节点表头
    for(int i = 0; i < graph.nodeNum; ++i){
        graph.adjList[i].nodeID = i;
        graph.adjList[i].outEdgeTails = nullptr;
        graph.adjList[i].inEdgeTails = nullptr;
        graph.adjList[i].edgeAttached = 0;
        graph.adjList[i].maxFlux = 0;
        graph.adjList[i].minSA = MY_INT_MAX;
    }
    //初始化超级结点信息
    graph.adjList[graph.nodeNum].nodeID = graph.nodeNum;    //源点
    graph.adjList[graph.nodeNum].outEdgeTails = nullptr;
    graph.adjList[graph.nodeNum].inEdgeTails = nullptr;
    graph.adjList[graph.nodeNum].edgeAttached = graph.custNum;
    graph.adjList[graph.nodeNum].maxFlux = BAND_MAX;

    graph.adjList[graph.nodeNum + 1].nodeID = graph.nodeNum + 1;    //汇点
    graph.adjList[graph.nodeNum + 1].outEdgeTails = nullptr;
    graph.adjList[graph.nodeNum + 1].inEdgeTails = nullptr;
    graph.adjList[graph.nodeNum + 1].edgeAttached = graph.custNum;
    graph.adjList[graph.nodeNum + 1].maxFlux = BAND_MAX;

    int ptrData = 4;    //消费结点数据开始的位置
    int a, b, c, d;     //单行网络信息：头节点，尾结点 带宽 费用
    stringstream charStream;
    //读取网络结点
    int RentSum = 0;
    int BandSum = 0;
    for(int i = 0; i < graph.edgeNum; ++i){
        charStream << topo[ptrData];
        //cout << "read Once..." << endl;
        //读入每条边的信息
        charStream >> a >> b >> c >> d;
        charStream.clear();
        ++ptrData;
        BandSum += c;
        RentSum += d;
        //更新结点个体信息
        graph.adjList[a].edgeAttached += 1;
        graph.adjList[b].edgeAttached += 1;
        graph.adjList[a].maxFlux += c;
        graph.adjList[b].maxFlux += c;

        Add_OneEdge(a, b, c, c, d);
        Add_OneEdge(b, a, c, c, d);
        //上行网络
//        EdgeNode* pEdge = new EdgeNode;
//        pEdge -> headID = a;
//        pEdge -> tailID = b;
//        pEdge -> bandWidth = c;
//        pEdge -> rest_bandWidth = c;
//        pEdge -> Rental = d;
//        pEdge -> vexOut = graph.adjList[a].outEdgeTails;
//
//        if(pEdge -> vexOut){
//            while(pEdge -> vexOut -> vexOut){
//               pEdge -> vexOut = pEdge -> vexOut -> vexOut;
//            }
//            pEdge -> vexOut -> vexOut = pEdge;
//            pEdge -> vexOut = nullptr;
//        }
//        else{
//             graph.adjList[a].outEdgeTails = pEdge;
//             pEdge -> vexOut = nullptr;
//        }
//        //下行网络
//        EdgeNode* pEdgeBack = new EdgeNode;
//        pEdgeBack -> headID = b;
//        pEdgeBack -> tailID = a;
//        pEdgeBack -> bandWidth = c;
//        pEdgeBack -> rest_bandWidth = c;
//        pEdgeBack -> Rental = d;
//        pEdgeBack -> vexOut = graph.adjList[b].outEdgeTails;
//
//        if(pEdgeBack -> vexOut){
//            while(pEdgeBack -> vexOut -> vexOut){
//                pEdgeBack -> vexOut = pEdgeBack -> vexOut -> vexOut;
//            }
//            pEdgeBack -> vexOut -> vexOut = pEdgeBack;
//            pEdgeBack -> vexOut = nullptr;
//        }
//        else{
//            graph.adjList[b].outEdgeTails = pEdgeBack;
//            pEdgeBack -> vexOut = nullptr;
//        }
    }
//    for(int i = 0;  i < graph.nodeNum; ++i){
//            EdgeNode **pInLink = &graph.adjList[i].inEdgeTails;
//            for(int j = 0; j < graph.nodeNum; ++j){
//                if(i == j){
//                    continue;
//                }
//                EdgeNode *p = graph.adjList[j].outEdgeTails;
//                while(p){
//                    if(p ->tailID != graph.adjList[i].nodeID){
//                        p = p ->vexOut;
//                        continue;
//                    }
//                    *pInLink = p;
//                    pInLink = &p ->vexIn;
//                    p = p ->vexOut;
//                }
//            }
//            *pInLink = nullptr;
//    }

    charStream.clear();
    ++ptrData;   //网络结点全部读完了，ptrData指向消费结点的开头
    graph.avgBand = BandSum / graph.edgeNum;
    graph.avgRent = RentSum / graph.edgeNum;
    //读取消费结点
    int sumNeed = 0;
    for(int j = 0; j < graph.custNum; ++j){
        charStream << topo[ptrData];
        charStream >> graph.customerInfo[j].customerID >> graph.customerInfo[j].toNodeID >> graph.customerInfo[j].bandNeed;
        charStream.clear();
        ++ptrData;
        sumNeed += graph.customerInfo[j].bandNeed;
    }
    graph.avgNeed = sumNeed / graph.custNum;
}

///删除拓扑图空间
void Destory_Graph(){
    for(int i = 0; i < graph.nodeNum + 2; ++i){
        EdgeNode *p = graph.adjList[i].outEdgeTails;
        EdgeNode *q;
        while(p){
            q = p;
            p = p -> vexOut;
            delete q;
        }
        graph.adjList[i].outEdgeTails = nullptr;
        graph.adjList[i].inEdgeTails = nullptr;
    }
    //Destory_SuperEndNodeEdge();
    //print_time("Destory Finish:");
}

///显示拓扑图
void Print_Graph(){
    cout << "----Print Nodes and Edges--------------------------" << endl;
    for(int i = 0; i < graph.nodeNum + 2; ++i){
        cout << graph.adjList[i].nodeID << "--out--(" << graph.adjList[i].edgeAttached << "," << graph.adjList[i].maxFlux << ")" << "-->";
        EdgeNode *p = graph.adjList[i].outEdgeTails;
        while(p){
            cout << "(" << p ->headID << ", " << p -> tailID << ", " << p -> bandWidth << ", " << p -> rest_bandWidth<< ", " << p -> Rental << ")";
            p = p -> vexOut;
        }
        cout << endl;
        p = graph.adjList[i].inEdgeTails;
        cout << graph.adjList[i].nodeID << "--in---(" << graph.adjList[i].edgeAttached << "," << graph.adjList[i].maxFlux << ")" << "-->";
        while(p){
            cout << "(" << p ->headID << ", " << p -> tailID << ", " << p -> bandWidth << ", " << p -> rest_bandWidth << ", " << p -> Rental << ")";
            p = p -> vexIn;
        }
        cout << endl << endl;
    }
    cout << "--Print customer message:" << endl;
    for(int i = 0; i < graph.custNum; ++i){
        cout << graph.customerInfo[i].customerID << "-->" << graph.customerInfo[i].toNodeID << ", Need:" << graph.customerInfo[i].bandNeed << endl;
    }
    cout << "--Other info:" << endl;
    cout << "--avgBand is " << graph.avgBand << endl;
    cout << "--avgRent is " << graph.avgRent << endl;
    cout << "--avgNeed is " << graph.avgNeed << endl;
    cout << "--serverCost is " << graph.serverCost << endl;
    cout << "----End Print N&E----" << endl << endl;
}

///返回边的已用带宽
int Check_Edge(int i, int j){
    EdgeNode *p = graph.adjList[i].outEdgeTails;
    while(p -> tailID != j){
        p = p ->vexOut;
    }
    if(!p){ //不存在该边
        return -123;
    }
    //cout << "Can find edge from " << p ->headID << "to" << p ->tailID << endl;
    return (p ->bandWidth - p ->rest_bandWidth);
}

///返回边的剩余带宽
int Check_Band(int i, int j){
    EdgeNode *p = graph.adjList[i].outEdgeTails;
    while(p -> tailID != j){
        p = p ->vexOut;
    }
    if(!p){ //不存在该边
        return -123;
    }
    return p ->rest_bandWidth;
}

///返回边的初始带宽
int Get_Band(int i, int j){
    EdgeNode *p = graph.adjList[i].outEdgeTails;
    while(p -> tailID != j){
        p = p ->vexOut;
    }
    if(!p){ //不存在该边
        return -123;
    }
    return p ->bandWidth;
}

///返回边的租金
int Get_Rent(int i, int j){
    EdgeNode *p = graph.adjList[i].outEdgeTails;
    while(p -> tailID != j){
        p = p ->vexOut;
    }
    if(!p){ //不存在该边
        return -123;
    }
    return p ->Rental;
}

