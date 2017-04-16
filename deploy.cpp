#include "deploy.h"
#include "limorton_SimAnnealing.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <iostream>
using namespace std;

void TimeIsUp(char * resultFilename);

void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * resultFilename){
    #if Lim_Debug
        print_time("startClock:");
    #endif // Lim_Debug
    startTime = clock();
    Init_Graph(topo);
    Create_Graph(topo);

//初始化服务器结点
    notServers = Find_Lonely_Node();    //元素为:不能成为服务器的结点编号
    beServers = Find_Be_Node();         //元素为:必须成为服务器的结点编号
    vector<int> beCustser = Find_Be_Cust(beServers);//custNum大小，元素为:1必须成为服务器
    #if Lim_Debug
        //cout << "Graph AVG: " << "band = " << graph.avgBand << ", need = " << graph.avgNeed << ", rent = " << graph.avgRent << endl;
        cout << "NOT servers at: ";
        for(u_int i = 0; i < notServers.size(); ++i){
            cout << " @" << notServers[i];
        }
        cout << endl << "Be servers at:";
        for(u_int i = 0; i < beServers.size(); ++i){
            cout << " @" << beServers[i];
        }
        cout << endl << "=====================================================================================" << endl;
    #endif // Lim_Debug

//    Destory_Graph();
//    system("pause");
//    exit(0);


    int startserveNum = 0;
    vector<int> startPos(graph.nodeNum, 0);
    int beNum = beServers.size();
    for(int i = 0; i < beNum; ++i){
        startPos[beServers[i]] = 2;
    }
    int notNum = notServers.size();
    for(int i = 0; i < notNum; ++i){
        startPos[notServers[i]] = -1;
    }

    #if 0
        Run_Dijkstra(beCustser);
        for(int i = 0; i < graph.custNum; ++i){
            if(beCustser[i] || graph.customerInfo[i].inPairs == true)
                continue;
            else{
                int minCost = MY_INT_MAX, minNode = i, custPair = -1;
                for(int j = 0; j < graph.nodeNum; ++j){
                    if(!NotCusttoNode(j) && j != graph.customerInfo[i].toNodeID \
                       && Min_Sum_Rent[j][i].rent < minCost && graph.customerInfo[Cust_Connect(j)].inPairs == false){
                        minNode = j;
                        custPair = Cust_Connect(j);
                    }
                }
                if(custPair == -1){ //跑完了， 还剩一个孤立结点
                    startPos[graph.customerInfo[i].toNodeID] = 1;
                    graph.adjList[i].setServer = true;
                    break;
                }
                graph.customerInfo[i].inPairs = true;
                graph.customerInfo[Cust_Connect(minNode)].inPairs = true;
                //cout << "Find pairs: " << i << " & " << Cust_Connect(minNode);
                //寻找结点 i 和 custPair 对的最近结点,设为服务器结点
                vector<int> RentPlus(graph.nodeNum);
                minCost = MY_INT_MAX;
                for(int j = 0; j < graph.nodeNum; ++j){
                    RentPlus[j] = Min_Sum_Rent[j][i].rent + Min_Sum_Rent[j][custPair].rent;
                    if(RentPlus[j] < minCost && graph.adjList[j].setServer == false && CountNearServer(j) <= 2){ //附近不能有超过一个已设置服务器结点
                        minCost = RentPlus[j];
                        minNode = j;
                    }
                }
                //cout << ", at " << minNode << endl;
                startPos[minNode] = 1;
                graph.adjList[minNode].setServer = true;
            }
            for(int j = 0; j < graph.nodeNum; ++j){
                if(startPos[j] > 0)
                    ++startserveNum;
            }
            if(startserveNum > graph.nodeNum * 13 / 16)
                break;
        }
        for(int i = 0; i < graph.custNum; ++i){
            if(graph.customerInfo[i].inPairs == false)
                startPos[graph.customerInfo[i].toNodeID] = 1;
        }
        //测试
        #if Lim_Debug && 0
            cout << "StartServNum = " << startserveNum << endl;
            DisPlayPos(startPos);
            Destory_Graph();
            exit(0);
        #endif // Lim_Debug
    #endif // 0

//SA
    #if 1
        if(graph.nodeNum < Junior_NUM){
            graph.guessSerNum = graph.custNum * 9 / 16;
            int Num = graph.guessSerNum - beNum;
            int rand01, number = 0;
            do{
                rand01 = rand() % graph.custNum;
                if(startPos[graph.customerInfo[rand01].toNodeID] == 0){
                    startPos[graph.customerInfo[rand01].toNodeID] = 1;
                    ++number;
                }

            }while(number < Num);
        }
        else if(graph.nodeNum < Medium_NUM){
            graph.guessSerNum = graph.custNum * 8.75 / 16;
            int Num = graph.guessSerNum - beNum;
            int rand01, number = 0;
            do{
                rand01 = rand() % graph.custNum;
                if(startPos[graph.customerInfo[rand01].toNodeID] == 0){
                    startPos[graph.customerInfo[rand01].toNodeID] = 1;
                    ++number;
                }

            }while(number < Num);
        }
        else{
            graph.guessSerNum = graph.custNum * 13 / 16;
            int Num = graph.guessSerNum - beNum;
            int rand01, number = 0;
            do{
                rand01 = rand() % graph.custNum;
                if(startPos[graph.customerInfo[rand01].toNodeID] == 0){
                    startPos[graph.customerInfo[rand01].toNodeID] = 1;
                    ++number;
                }

            }while(number < Num);
        }
    #else
        for(int i = 0; i < graph.custNum; ++i){ //服务器位置初始设在消费节点所连网络节点
            if(startPos[graph.customerInfo[i].toNodeID] == 0){
                startPos[graph.customerInfo[i].toNodeID] = 1;
            }
        }
    #endif // Lim_Debug

    startserveNum = 0;
    for(int i = 0; i < graph.nodeNum; ++i){
        if(startPos[i] > 0)
            ++startserveNum;
    }
    graph.guessSerNum = startserveNum;
    #if Lim_Debug
            cout << "guessSerNum = " << startserveNum << endl;
    #endif // Lim_Debug


    Add_SuperEndNodeEdge();
    SimAnneal(startPos);
    TimeIsUp(resultFilename);
    return;
}

void TimeIsUp(char * resultFilename){
    #if 0
        cout << "===============================================" << endl;
        print_time("Write to file:");
    #endif // Lim_Debug
    //Run_MCMF(SAParms.PreGlbPos, true);
    Run_MCMF(SAParms.GlbBestPos, true);
    //写入文件
	#if !Lim_Debug
        int routeNum = ANS_ROUTE.size();
        inStream << routeNum;
        inStream >> str_output;
        inStream.clear();
        str_output += "\n\n";
        for(int i = 0; i < routeNum; ++i){
            ///查 STL算法中 copy 的用法
    //        copy(ANS_ROUTE[i].begin(), ANS_ROUTE[i].end(), ostream_iterator<int>(inStream, " "));
    //        inStream >> str_output;
    //        inStream.clear();
    //        str_output += "\n";
            //----------------------
            int Num = ANS_ROUTE[i].size();
            for(int j = 0; j < Num; ++j){
                inStream << ANS_ROUTE[i][j];
                inStream >> str_temp;
                inStream.clear();
                str_output += str_temp;
                if(j < Num -1)
                    str_output += " ";
                else
                    str_output += "\n";
            }
        }
    #endif // Lim_Debug

	#if Lim_Debug
        currentTime = clock();
        cout << "### Total Time: " << (double)(currentTime - startTime) / CLOCKS_PER_SEC << endl;
        cout << "============================================================================" << endl;
	#endif // Lim_Debug

	char * topo_file = (char*) str_output.data();
	//将result缓冲区中的内容写入文件，写入方式为覆盖写入
	write_result(topo_file, resultFilename);
	Destory_Graph();
	#if Lim_Debug
        system("pause");
	#endif // Lim_Debug
	return;
}
