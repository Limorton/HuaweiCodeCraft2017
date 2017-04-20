#ifndef LIMORTON_GRAPH_H_INCLUDED
#define LIMORTON_GRAPH_H_INCLUDED
#include <vector>
#include <limits.h>

#define Lim_Debug 1

using namespace std;

typedef unsigned int u_int;

const int BAND_MAX = INT_MAX;
const int DIST_MAX = INT_MAX;
const int FLOW_MAX = INT_MAX;
const int COST_MAX = INT_MAX;
const int MY_INT_MAX = INT_MAX;

#if Lim_Debug
    static int serverNum_err = 0;
    static int edgeband_err = 0;
    static int routeNeed_err = 0;
#endif // Lim_Debug


//图的存储结构
typedef int NodeType;        //结点编号类型
typedef int BandWidthType;   //边带宽类型
typedef int RentType;       //单位带宽租用费类型
typedef int CustomerType;   //消费结点编号类型
typedef int NeedType;       //消费结点需求类型

/** \brief 边结构
 */
struct EdgeNode{
    NodeType headID;    //本条边的头结点
    NodeType tailID;    //本条边的尾结点

    BandWidthType bandWidth;        //网络带宽，在 1-100之间
    BandWidthType rest_bandWidth;   //已用网络带宽，在 1-100之间
    RentType Rental;                //单位带宽租用费，在 1-100之间

    EdgeNode *vexOut;   //指向头节点相同的下条边
    EdgeNode *vexIn;    //指向尾节点相同的下条边
};

/** \brief 结点结构
 */
struct VertexNode{
    NodeType nodeID;      //网络结点编号，结点最多有1000个

    int edgeAttached;       //与结点相连的边的条数，最大为20
    int maxFlux;           //所有相连的边的带宽的和，若该结点连接消费结点，且 maxFlux < Need, 则必须在该结点放置服务器
    //Dijkstra参数
    int sum_rent;           //源结点到此结点最短链路的租金和
    bool setServer;         //是否已设服务器
    //SA参数
    int minSA;              //当前SA算法在此处设服务器的最小花费

    EdgeNode* inEdgeTails;  //前驱结点链表
    EdgeNode* outEdgeTails; //后继结点链表
};

/** \brief 消费节点
 */
struct Customer{
    CustomerType customerID;    //消费结点编号，不超过500个
    NodeType toNodeID;      //连接的网络结点标号
    NeedType bandNeed;      //带宽需求，在 1-5000 之间
    //Dijkstra参数
    bool inPairs;           //已配对
};

/** \brief 全图结构
 */
struct VideoNetGraph{
    int nodeNum;
    int edgeNum;
    int custNum;
    int serverCost;
    int avgRent;
    int avgBand;
    int avgNeed;
    int guessSerNum;
    int dirCost;

    vector<VertexNode> adjList;
    vector<Customer> customerInfo;
};

extern VideoNetGraph graph;

/** \brief
 *  输出显示结点i的出边
 * \param i int
 * \return void
 *
 */
void Print_Edges(int i);
/** \brief
 *  初始拓扑图
 * \param topo char**--输入文件
 * \return void
 *
 */
void Init_Graph(char** topo);
/** \brief
 *  向图中添加一条边
 * \param head int--头节点
 * \param tail int--尾结点
 * \param band int--带宽
 * \param r_band int--剩余带宽
 * \param rent int--租金
 * \return void
 *
 */
void Add_OneEdge(int head, int tail, int band, int r_band, int rent);
/** \brief
 *  从图中删除一条边
 * \param head int--头节点
 * \param tail int--尾结点
 * \return void
 *
 */
void Delete_OneEdge(int head, int tail);
/** \brief
 *  添加超级源边
 * \param ServerNode const vector<int>&--设置服务器的结点集合
 * \return void
 *
 */
void Add_SuperSourceNodeEdge(const vector<int>& ServerNode);
/** \brief
 *  删除超级源边
 * \param ServerNode const vector<int>&--设置服务器的结点集合
 * \return void
 *
 */
void Destory_SuperSourceNodeEdge(const vector<int>& ServerNode);
/** \brief
 *  添加超级汇边
 * \return void
 *
 */
void Add_SuperEndNodeEdge();
/** \brief
 *  删除超级汇边
 * \return void
 *
 */
void Destory_SuperEndNodeEdge();
/** \brief
 *  生成拓扑图
 * \param topo char**--输入文件
 * \return void
 *
 */
void Create_Graph(char** topo);
/** \brief
 *  删除拓扑图空间
 * \return void
 *
 */
void Destory_Graph();
/** \brief
 *  显示拓扑图
 * \return void
 *
 */
void Print_Graph();
/** \brief
 *  返回边的已用带宽
 * \param i int--头节点
 * \param j int--尾结点
 * \return int--已用带宽
 *
 */
int Check_Edge(int i, int j);
/** \brief
 *  返回边的剩余带宽
 * \param i int--头节点
 * \param j int--尾结点
 * \return int--边的剩余带宽
 *
 */
int Check_Band(int i, int j);
/** \brief
 *  返回边的初始带宽
 * \param i int--头节点
 * \param j int--尾结点
 * \return int--边的初始带宽
 *
 */
int Get_Band(int i, int j);
/** \brief
 *  返回边的租金
 * \param i int--头节点
 * \param j int--尾结点
 * \return int--边租金
 *
 */
int Get_Rent(int i, int j);

#endif // LIMORTON_GRAPH_H_INCLUDED
