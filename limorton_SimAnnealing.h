#ifndef LIMORTON_SIMANNEALING_H_INCLUDED
#define LIMORTON_SIMANNEALING_H_INCLUDED
#include "limorton_MCMF.h"
#include "limorton_dijkstra.h"
#include <vector>
#include <iostream>
#include <sstream>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <stdlib.h>
#include <time.h>

using namespace std;

//哈希表存访问过的状态
static unordered_set<string> visitedState;
static bool TimeUp = false;
static clock_t startTime;
static clock_t currentTime;
static double endTime;
const int Junior_NUM = 250;
const int Medium_NUM = 550;

static stringstream inStream;
static string str_output = "";
static string str_temp = "";
/************************************************************************/
static double T_0;
static double ALPHA_SA;
static double BETA_SA;      //回温因子
static double Kb;

static double BETA_pMinSA;    //选l邻近最小minSA的概率因子

static int STEP_U;
static int Step_1;
static int stay_Max_1;
static double P_PICK_1;
static int inLoop1;

static int Step_2;
static int stay_Max_2;
static double P_PICK_2;
static int inLoop2;

static int Step_0;
static int stay_Max_0;
static double P_PICK_0;
static int inLoop0;
static int stay_Max_Final;
static int changeKind0;
static int changeKind1;
static int changeKind2;
static int changeNodeNumber;
static int changeNodeNum0;
static int changeNodeNum1;
static int changeNodeNum2;
static double P_pickMinSA;

static int maxVisitedState;
/**************************************************************************/
struct AnnealParm{
    double Temperature;      //温度,越高，接受”坏解“的能力越强
    int GlbBestFx;           //全局最优解
    int PreBestFx;
    int OldBestFx;           //当前目标函数
    int NewBestFx;
    vector<int> GlbBestPos;  //全局最优解位置
    vector<int> OldPos;     //当前解位置
    vector<int> NewPos;     //下一个解位置
    vector<int> PreGlbPos;
};
static vector<int> changedPosition;
static AnnealParm SAParms;
static double p_metro;        //Metropolis概率
static int SAstep = 1;     //总迭代次数
static int steps;           //单次SA迭代次数
static int stays_loc = 0;      //单次陷入局部最优解后的迭代次数
static int stays_glb = 0;
#if Lim_Debug
    static int firstFind = 0;  //第一次发现全局最优解的迭代次数
    static double findTime;
    //static int notVisited;
#endif // Lim_Debug
/****************************************************/
void Set_Junior(){
    T_0 = 100;
    ALPHA_SA = 0.99;
    Kb = 0.99;
    BETA_SA = 0.5;  //回温

    P_PICK_0 = 0.35;
    P_PICK_1 = 0.40;    //起始选邻近最小minSA概率
    P_PICK_2 = 0.30;
    BETA_pMinSA = 1;  //选邻近解因子

    inLoop0 = 6;
    inLoop1 = 4;    //影响总SAstep和降温速度
    inLoop2 = 2;     //大inLoop和小step(1)配合实现局部搜索

    changeKind0 = 2;
    changeKind1 = 2;
    changeKind2 = 3; changeNodeNum2 = 2;
    Step_0 = 6;
    Step_1 = 3;
    Step_2 = 4;

    stay_Max_0 = 700;
    stay_Max_1 = 400;   //过大则不能发挥反复回温的优势
    stay_Max_2 = 300;
    stay_Max_Final = 3000;

    maxVisitedState = 200;

    #if Lim_Debug
        endTime = 65;
    #else
        endTime = 89.0;
    #endif // Lim_Debug
}
/****************************************************/
void Set_Medium(){
    T_0 = 110;
    ALPHA_SA = 0.998;
    Kb = 0.97;
    BETA_SA = 0.5;  //回温

    P_PICK_0 = 0.35;
    P_PICK_1 = 0.35;    //起始选邻近最小minSA概率
    P_PICK_2 = 0.35;
    BETA_pMinSA = 1;  //选邻近解因子

    inLoop0 = 4;
    inLoop1 = 5;
    inLoop2 = 3;

    changeKind0 = 3; changeNodeNum0 = 3;
    changeKind1 = 2; changeNodeNum1 = 2;
    changeKind2 = 3; changeNodeNum2 = 2;
    Step_0 = 6;
    Step_1 = 4;
    Step_2 = 3;

    stay_Max_0 = 40;
    stay_Max_1 = 50;
    stay_Max_2 = 50;
    stay_Max_Final = 2000;

    maxVisitedState = 200;

    #if Lim_Debug
        endTime = 65;
    #else
        endTime = 88.7;
    #endif // Lim_Debug
}
/****************************************************/
void Set_Senior(){
    T_0 = 70;
    ALPHA_SA = 0.996;
    Kb = 0.97;
    BETA_SA = 0.6;  //回温

    P_PICK_0 = 0.2;
    P_PICK_1 = 0.15;    //选邻近最小minSA概率
    P_PICK_2 = 0.2;
    BETA_pMinSA = 1;  //选邻近解因子

    inLoop0 = 8;   //越大初期降得越快
    inLoop1 = 7;
    inLoop2 = 6;

    changeKind0 = 3; changeNodeNum0 = 6;
    changeKind1 = 3; changeNodeNum1 = 10;
    changeKind2 = 3; changeNodeNum2 = 5;
    Step_0 = 10;
    Step_1 = 8;
    Step_2 = 4;

    stay_Max_0 = 20;
    stay_Max_1 = 10;
    stay_Max_2 = 20;
    stay_Max_Final = 2000;

    maxVisitedState = 200;

    #if Lim_Debug
        endTime = 65;
    #else
        endTime = 88.5;
    #endif // Lim_Debug
}
/****************************************************/
///计算仍未访问过的结点的数量
int CountNotVisited(){
    int countNum = 0;
    for(int i = 0; i < graph.nodeNum; ++i){
        if(graph.adjList[i].minSA == MY_INT_MAX)
            ++countNum;
    }
    return countNum;
}

#if Lim_Debug
void DisPlayPos(const vector<int>& vec){
    int vecSize = vec.size();
    cout << "#OServer at:" << endl;
    for(int i = 0; i < vecSize; ++i){
        if(vec[i] == 1 || vec[i] == 2)
            cout<< "_" << i;
    }
    cout << endl;
}
#endif // Lim_Debug

#if Lim_Debug
void DisPlayPos01(const vector<int>& vec){
    int vecSize = vec.size();
    cout << "#Server at:" << endl;
    for(int i = 0; i < vecSize; ++i){
        cout<< "(" << i << ", " << vec[i] << ")";
    }
    cout << endl;
}
#endif // Lim_Debug

///求2的n次幂：STL--- pow(2, n);
int pow2n(int n){
    int ans = 1;
    while(--n)
        ans *=2;
    return 2 * ans;
}
///vector<int> 转 int
int toInt(const vector<int>& vec){
    int sum = 0;
    for(unsigned int i = 0; i < vec.size(); ++i){
        if(vec[i] == 1)
            sum += pow2n(vec.size() - i - 1);
    }
    return sum;
}
///vector<int> 转 string
string vecToString(const vector<int>& vec){
    string str, str_temp;
    str.resize(vec.size());
    int num = vec.size();
    for(int i = 0; i < num; ++i){
        str[i] = (char)vec[i];
    }
    return str;
}

///初始解空间
void Init_AnsSpace(const vector<int>& startState){
    SAParms.Temperature = T_0;
    SAParms.OldPos.assign(graph.nodeNum, 0);
    SAParms.GlbBestPos.assign(graph.nodeNum, 0);
    SAParms.NewPos.assign(graph.nodeNum, 0);

    SAParms.OldPos = startState;
    SAParms.GlbBestPos = startState;
    SAParms.GlbBestFx = Run_MCMF(SAParms.OldPos, false);
    SAParms.OldBestFx = SAParms.GlbBestFx;
    SAParms.PreBestFx = SAParms.GlbBestFx;
    SAParms.PreGlbPos = SAParms.GlbBestPos;
}

///状态产生函数1: stepU个结点移动一位
int NewState1(const vector<int>& notServers, const vector<int>& beServers){
    int beNum = beServers.size();
    for(int i = 0; i < beNum; ++i){
        SAParms.NewPos[beServers[i]] = 2;
    }
    int notNum = notServers.size();
    for(int i = 0; i < notNum; ++i){
        SAParms.NewPos[notServers[i]] = -1;
    }
    vector<int> OldPos = SAParms.OldPos;
    for(int e = 0; e < STEP_U; ++e){
        SAParms.NewPos = OldPos;
        int changeID = 0;
        int positionID = 0, count_1 = -1;
        double P_randpick;
        int random = rand() % (graph.guessSerNum - beNum);    //随机选择结点
        for(int i = 0; i < graph.nodeNum; ++i){
            if(OldPos[i] == 1){
                ++count_1;
            }
            if(count_1 == random){
                positionID = i;
                break;
            }
        }

        P_randpick = rand() % 100/(double)101;
        bool noBetter = true;
        if(P_randpick < P_pickMinSA){    //以概率P_PICK选择邻接“最好”的0结点
            EdgeNode *p = graph.adjList[positionID].outEdgeTails;
            int min_SA = graph.adjList[positionID].minSA;
            while(p && (p ->tailID < graph.nodeNum)){
                if((graph.adjList[p ->tailID].minSA <= min_SA) && (OldPos[p ->tailID] == 0)){
                    min_SA = graph.adjList[p ->tailID].minSA;
                    changeID = p ->tailID;
                    noBetter = false;
                }
                p = p ->vexOut;
            }
            //因为可能在周围找不到比他还小的点，所以changeID = 0; 出错
            if(noBetter == false){
                SAParms.NewPos[positionID] = 0;
                SAParms.NewPos[changeID] = 1;
                OldPos = SAParms.NewPos;
            }
        }
        if((P_randpick >= P_pickMinSA) || noBetter){    //随机选邻近的结点交换0，1
            count_1 = -1;
            if(graph.nodeNum >= graph.custNum){
                EdgeNode *p = graph.adjList[positionID].outEdgeTails;
                int count0Edge = 0;
                while(p && (p ->tailID < graph.nodeNum)){ //计算临近点中0的个数
                    if(OldPos[p ->tailID] == 0){
                        ++count0Edge;
                    }
                    p = p ->vexOut;
                }
                //可能没有0，所以count0Edge = 0; 出错
                if(count0Edge !=0){
                    int random2 = rand() % (count0Edge);    //产生一个0~count0Edge之间随机数
                    p = graph.adjList[positionID].outEdgeTails;
                    while(p && (p ->tailID < graph.nodeNum)){
                        if(OldPos[p ->tailID] == 0){
                            ++count_1;
                        }
                        if(count_1 == random2){
                            changeID = p ->tailID;
                            break;
                        }
                        p = p ->vexOut;
                    }
                    SAParms.NewPos[positionID] = 0;
                    SAParms.NewPos[changeID] = 1;
                    OldPos = SAParms.NewPos;
                }
            }
            else{
                cout << "----nodeNum < custNum----" << endl;
                Destory_Graph();
                exit(0);
            }
        }
    }
    return 0;
}
///状态产生函数2: 一个结点移动stepU位
int NewState2(const vector<int>& notServers, const vector<int>& beServers){
    int beNum = beServers.size();
    for(int i = 0; i < beNum; ++i){
        SAParms.NewPos[beServers[i]] = 2;
    }
    int notNum = notServers.size();
    for(int i = 0; i < notNum; ++i){
        SAParms.NewPos[notServers[i]] = -1;
    }

    int forbideState = 0;
    bool Visited = false;
    int positionID = 0, count_1 = -1;
    vector<int> OldPos = SAParms.OldPos;
    do{
        Visited = false;
        int random = rand() % (graph.guessSerNum - beNum);    //随机选择结点
        for(int i = 0; i < graph.nodeNum; ++i){
            if(OldPos[i] == 1){
                ++count_1;
            }
            if(count_1 == random){
                positionID = i;
                break;
            }
        }
        for(int e = 0; e < STEP_U; ++e){
            SAParms.NewPos = OldPos;
            int changeID = 0;
            double P_randpick = rand() % 100/(double)101;
            bool noBetter = true;
            if(P_randpick < P_pickMinSA){    //以概率P_PICK选择邻接“最好”的0结点
                EdgeNode *p = graph.adjList[positionID].outEdgeTails;
                int min_SA = graph.adjList[positionID].minSA;
                while(p && (p ->tailID < graph.nodeNum)){
                    if((graph.adjList[p ->tailID].minSA <= min_SA) && (OldPos[p ->tailID] == 0)){
                        min_SA = graph.adjList[p ->tailID].minSA;
                        changeID = p ->tailID;
                        noBetter = false;
                    }
                    p = p ->vexOut;
                }
                //因为可能在周围找不到比他还小的点，所以changeID = 0; 出错
                if(noBetter == false){
                    SAParms.NewPos[positionID] = 0;
                    SAParms.NewPos[changeID] = 1;
                    OldPos = SAParms.NewPos;
                }
            }

            if((P_randpick >= P_pickMinSA) || noBetter){    //随机选邻近的结点交换0，1
                count_1 = -1;
                if(graph.nodeNum >= graph.custNum){
                    EdgeNode *p = graph.adjList[positionID].outEdgeTails;
                    int count0Edge = 0;
                    while(p && (p ->tailID < graph.nodeNum)){ //计算临近点中0的个数
                        if(OldPos[p ->tailID] == 0){
                            ++count0Edge;
                        }
                        p = p ->vexOut;
                    }
                    //可能没有0，所以count0Edge = 0; 出错
                    if(count0Edge !=0){
                        int random2 = rand() % (count0Edge);
                        p = graph.adjList[positionID].outEdgeTails;
                        while(p && (p ->tailID < graph.nodeNum)){
                            if(OldPos[p ->tailID] == 0){
                                ++count_1;
                            }
                            if(count_1 == random2){
                                changeID = p ->tailID;
                                break;
                            }
                            p = p ->vexOut;
                        }
                        SAParms.NewPos[positionID] = 0;
                        SAParms.NewPos[changeID] = 1;
                        OldPos = SAParms.NewPos;
                    }
                }
                else{
                    cout << "----nodeNum < custNum----" << endl;
                    Destory_Graph();
                    exit(0);
                }
            }
            positionID = changeID;
        }
        if(visitedState.find(vecToString(SAParms.NewPos)) != visitedState.end()){
            //cout << "visited state" << endl;
            ++forbideState;
            Visited = true;
        }
        if(forbideState > maxVisitedState)
            break;
    }while(Visited);
    visitedState.insert(vecToString(SAParms.NewPos));
    if(forbideState > maxVisitedState)
        return -1;
    else
        return 0;
}

///状态产生函数3: 多个结点移动stepU位
int NewState3(const vector<int>& notServers, const vector<int>& beServers){
    int beNum = beServers.size();
    for(int i = 0; i < beNum; ++i){
        SAParms.NewPos[beServers[i]] = 2;
    }
    int notNum = notServers.size();
    for(int i = 0; i < notNum; ++i){
        SAParms.NewPos[notServers[i]] = -1;
    }

    int forbideState = 0;
    bool Visited = false;
    do{
        Visited = false;
        for(int i = 0; i < changeNodeNumber; ++i){
            int positionID = 0, count_1 = -1;
            vector<int> OldPos = SAParms.OldPos;
            int random = rand() % (graph.guessSerNum - beNum);    //随机选择结点
            for(int i = 0; i < graph.nodeNum; ++i){
                if(OldPos[i] == 1){
                    ++count_1;
                }
                if(count_1 == random){
                    positionID = i;
                    break;
                }
            }
            for(int e = 0; e < STEP_U; ++e){
                SAParms.NewPos = OldPos;
                int changeID = 0;
                double P_randpick = rand() % 100/(double)101;
                bool noBetter = true;
                if(P_randpick < P_pickMinSA){    //以概率P_PICK选择邻接“最差”的0结点
                    EdgeNode *p = graph.adjList[positionID].outEdgeTails;
                    int min_SA = graph.adjList[positionID].minSA;
                    while(p && (p ->tailID < graph.nodeNum)){
                        if((graph.adjList[p ->tailID].minSA <= min_SA) && (OldPos[p ->tailID] == 0)){
                            min_SA = graph.adjList[p ->tailID].minSA;
                            changeID = p ->tailID;
                            noBetter = false;
                        }
                        p = p ->vexOut;
                    }
                    //因为可能在周围找不到比他还小的点，所以changeID = 0; 出错
                    if(noBetter == false){
                        SAParms.NewPos[positionID] = 0;
                        SAParms.NewPos[changeID] = 1;
                        OldPos = SAParms.NewPos;
                    }
                }

                if((P_randpick >= P_pickMinSA) || noBetter){    //随机选邻近的结点交换0，1
                    count_1 = -1;
                    if(graph.nodeNum >= graph.custNum){
                        EdgeNode *p = graph.adjList[positionID].outEdgeTails;
                        int count0Edge = 0;
                        while(p && (p ->tailID < graph.nodeNum)){ //计算临近点中0的个数
                            if(OldPos[p ->tailID] == 0){
                                ++count0Edge;
                            }
                            p = p ->vexOut;
                        }
                        //可能没有0，所以count0Edge = 0; 出错
                        if(count0Edge !=0){
                            int random2 = rand() % (count0Edge);
                            p = graph.adjList[positionID].outEdgeTails;
                            while(p && (p ->tailID < graph.nodeNum)){
                                if(OldPos[p ->tailID] == 0){
                                    ++count_1;
                                }
                                if(count_1 == random2){
                                    changeID = p ->tailID;
                                    break;
                                }
                                p = p ->vexOut;
                            }
                            SAParms.NewPos[positionID] = 0;
                            SAParms.NewPos[changeID] = 1;
                            OldPos = SAParms.NewPos;
                        }
                    }
                    else{
                        cout << "----nodeNum < custNum----" << endl;
                        Destory_Graph();
                        exit(0);
                    }
                }
                positionID = changeID;
            }
        }
        if(visitedState.find(vecToString(SAParms.NewPos)) != visitedState.end()){
            ++forbideState;
            //cout << "visited state" << endl;
            Visited = true;
        }
        if(forbideState > maxVisitedState)
            break;
    }while(Visited);
    visitedState.insert(vecToString(SAParms.NewPos));
    if(forbideState > maxVisitedState)
        return -1;
    else
        return 0;
}

///单次模拟退火
void OneSA(int stepU, int stayMax, int loop, int t_start, double pick_start, int changeKind, int changeNum){
    steps = 0;
    stays_loc = 0;
    STEP_U = stepU;
    changeNodeNumber = changeNum;
    SAParms.Temperature = t_start;
    while(stays_loc < stayMax){
        ++SAstep;
        ++steps;
        //P_pickMinSA = pick_start + (1 - pick_start) * (1 - pow(BETA_pMinSA, steps));        ///选临近最小minSA结点概率公式，用于变为1
        P_pickMinSA = pick_start;
        bool allVisited = false;
        for(int i = 0; i < loop; ++i){
            int localvisited;
            if(changeKind == 1){
                localvisited = NewState1(notServers, beServers);
            }
            else if(changeKind == 2){
                localvisited = NewState2(notServers, beServers);
            }
            else{
                localvisited = NewState3(notServers, beServers);
            }
            if(localvisited == -1){
                allVisited = true;
                break;
            }

            #if 0
                CheckOK(SAParms.NewPos, notServers, beServers);
            #endif // Lim_Debug
            SAParms.NewBestFx = Run_MCMF(SAParms.NewPos, false);
            for(int j = 0; j < graph.nodeNum; ++j){ //更新结点minSA
                if((SAParms.NewPos[j] > 0) && (graph.adjList[j].minSA > SAParms.NewBestFx)){
                    graph.adjList[j].minSA = SAParms.NewBestFx;
                }
            }
            if(SAParms.NewBestFx < SAParms.OldBestFx ){ //按Metropolis概率准则接受新解
                SAParms.OldBestFx = SAParms.NewBestFx;
                SAParms.OldPos = SAParms.NewPos;
            }
            else{
                p_metro = exp((SAParms.OldBestFx \
                        - SAParms.NewBestFx) / (Kb * SAParms.Temperature));                 ///Metropolis 准则
                double randNum = rand() % 100/(double)101;
                if(randNum < p_metro){  //接受坏解解
                    SAParms.OldBestFx = SAParms.NewBestFx;
                    SAParms.OldPos = SAParms.NewPos;
                }
            }
        }
        if(allVisited == true)
            break;
        if(SAParms.OldBestFx < SAParms.GlbBestFx){  //更新最优解
            SAParms.PreBestFx = SAParms.GlbBestFx;
            SAParms.PreGlbPos = SAParms.GlbBestPos;
            SAParms.GlbBestFx = SAParms.OldBestFx;
            SAParms.GlbBestPos = SAParms.OldPos;
            stays_loc = 0;
            int curSerNum = Cal_ServersNum();
            #if Lim_Debug
                firstFind = SAstep;
                currentTime = clock();
                findTime = (double)(currentTime - startTime) / CLOCKS_PER_SEC;
                cout << "*Step " << SAstep << ": " << SAParms.OldBestFx << " / " << graph.dirCost \
                    << ", P_0 = " << P_pickMinSA << ", P_e = " << p_metro << ", T =  " << SAParms.Temperature;
                currentTime = clock();
                cout << ", Find Time:" << (double)(currentTime - startTime) / CLOCKS_PER_SEC;
                if(curSerNum < graph.guessSerNum)
                    cout << " ---guessNum= " << curSerNum << endl;
                else
                    cout << endl;
                //cout << "notVisited = " << CountNotVisited() << endl;
            #endif // Lim_Debug


            if(curSerNum < graph.guessSerNum){
                graph.guessSerNum = curSerNum;
                for(int j = 0; j < graph.nodeNum; ++j){
                    if(SAParms.OldPos[j] == 1)
                        SAParms.OldPos[j] = 0;
                }
                int routeNum = ANS_ROUTE.size();
                for(int j = 0; j < routeNum; ++j){
                    if(SAParms.OldPos[ANS_ROUTE[j][0]] == 2)
                        continue;
                    else
                        SAParms.OldPos[ANS_ROUTE[j][0]] = 1;
                }
            }
            #if Lim_Debug
                //写入输出流
                inStream << (double)(currentTime - startTime);
                inStream >> str_temp;
                inStream.clear();
                str_output += str_temp + " ";
                inStream << SAParms.GlbBestFx;
                inStream >> str_temp;
                inStream.clear();
                str_output += str_temp + "\n";
            #endif // Lim_Debug
        }
        else{
            ++stays_loc;
            ++stays_glb;
        }

        //SAParms.Temperature = T_0 * exp(- ALPHA_SA * sqrt(steps - 1));
        SAParms.Temperature *= ALPHA_SA;                                                        ///T 更新公式

        currentTime = clock();
        if((double)(currentTime - startTime) / CLOCKS_PER_SEC > endTime){
            #if Lim_Debug
                cout << "**Time is Up!" << endl;
            #endif // Lim_Debug
            TimeUp = true;
            break;
        }
    }
}

///SA过程
void SimAnneal(const vector<int>& startState){
    srand(time(0));
    #if Lim_Debug
        cout.precision(4);
        cout << "CustNum = " << graph.custNum << endl;
    #endif // Lim_Debug
    Init_AnsSpace(startState);
    if(graph.nodeNum < Junior_NUM){
        Set_Junior();
    }else if(graph.nodeNum < Medium_NUM){
        Set_Medium();
    }else{
        Set_Senior();
    }
    //过程零：模拟退火  快速降低最优解
    SAParms.OldPos = SAParms.GlbBestPos;
    SAParms.OldBestFx = SAParms.GlbBestFx;
    OneSA(Step_0, stay_Max_0, inLoop2, T_0, P_PICK_0, changeKind0, changeNodeNum0);
    #if Lim_Debug
        cout << "0 Over";
        cout << "=====================================================================================" << endl;
    #endif // Lim_Debug
    srand(time(0));
    if(TimeUp)
        return;
    int backCount = 1;
    while(stays_glb < stay_Max_Final){
        if(TimeUp)
            break;
        //过程一：回火升温 跳出局部最优解
        SAParms.OldPos = SAParms.GlbBestPos;
        SAParms.OldBestFx = SAParms.GlbBestFx;
        OneSA(Step_1, stay_Max_1, inLoop0, pow(BETA_SA, backCount) * T_0, P_PICK_1, changeKind1, changeNodeNum1);   ///回温公式
        #if Lim_Debug
            cout << "1 Over";
            cout << "=====================================================================================" << endl;
        #endif // Lim_Debug
        if(TimeUp)
            break;
        //过程二：模拟退火  全局搜索最优解
        SAParms.OldPos = SAParms.GlbBestPos;
        SAParms.OldBestFx = SAParms.GlbBestFx;
        OneSA(Step_2, stay_Max_2, inLoop2, pow(BETA_SA, backCount) * BETA_SA * T_0, P_PICK_2, changeKind2, changeNodeNum2);
        #if Lim_Debug
            cout << "2 Over";
            cout << "=====================================================================================" << endl;
        #endif // Lim_Debug
        ++backCount;
    }
    #if Lim_Debug
        cout << "### CustNeed_err = " << routeNeed_err << "\n";
        cout << "### Find At Time:  " << findTime << " s" << endl;
    #endif // Lim_Debug
}
#endif // LIMORTON_SIMANNEALING_H_INCLUDED

