#pragma once
#include "GlobalParameters.h"
#include "Topology.h"

/*
生成路径时候一些注意事项，如果要调用InsertAfter()，请务必让当前路径保持一个标准
状态（即：sink开头，sink结尾）。因为InsertAfter()中每次插入之后会更新路径信息，
这就要求路径是满足0开头，0结尾的结构。否则有可能会出错
*/


class CRoute
{
public:
	CRoute(void);
	~CRoute(void);
	CRoute(BASIC_ROUTE_Struct route);
	
	/*The Only Attribute*/
	BASIC_ROUTE_Struct current_route;

	/*路径可行性检验：接收一个POI的id作为参数，检验将POI插入路径是否可行 */
	//只检查当前路径的capacity是否可以满足POI的demand
	bool IsFeasible(int poi_id);//计算是否可行（仅检测buffer）,这个函数可以根据不同的要求改写其校验规则
	bool IsFeasibleTimeAndBuffer(int poi_id);//计算是否可行（检测buffer和时间）

	/*根据current_route.seq更新current_route的其他成员，即遍历current_route的访问序列，计算路径总长度，路径上POI个数（不包括sink），路径所需节点数，和路径capacity*/
	void UpdateStatus();

	/*打印当前路径*/
	void PrintRoute();

	/*针对路径的优化操作，可不断添加*/
	//2opt
	BASIC_ROUTE_Struct Two_Opt(BASIC_ROUTE_Struct*brs);

	/*路径相关的插入与删除操作，在插入操作前必须调用IsFeasible来检查能否进行插入，在所有操作结束前必须调用updatestatus*/
	/*
	called by create_Initializeial_solution and may be called by Neighbor();
	insert p(id) into p(pos).This function will update current_route;
	*/
	/*
	生成路径时候一些注意事项，如果要调用InsertAfter()，请务必让当前路径保持一个标准
	状态（即：sink开头，sink结尾）。因为InsertAfter()中每次插入之后会更新路径信息，
	这就要求路径是满足sink开头，sink结尾的结构。否则有可能会出错
	*/
	bool InsertAfter(int pos,int id);//将当前节点号为id的节点，插入在节点号为pos的POI节点之后

	/*
	maybe called by Neighbor()
	This function will update current_route;
	*/
	bool DeleteFromRoute(int id);//删除节点号为id的POI节点

	bool SolomenInsert(int id);//找当前路径最小增量的地方插入标号为id的节点

	bool Is_Empty();//当前路径是否是只有两个sink节点或其他出错信息
};