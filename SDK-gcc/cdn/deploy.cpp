#include "deploy.h"
#include <stdio.h>

// 你要完成的功能总入口
// * topo[MAX_EDGE_NUM]  每一个元素是一个字符指针，对应文件中一行的内容
// line_num  文件的行数
// resultFilename  输出文件名
void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * resultFilename)
{
    // 处理topo[MAX_EDGE_NUM]，生成图


	// 需要输出的内容
	char * topo_file = (char *)" ";

	//将result缓冲区中的内容写入文件，写入方式为覆盖写入
	// 直接调用输出文件的方法输出到指定文件中(ps请注意格式的正确性，如果有解，第一行只有一个数据；第二行为空；第三行开始才是具体的数据，数据之间用一个空格分隔开)
	write_result(topo_file, filename);

}
