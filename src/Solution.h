#pragma once
#include "GlobalParameters.h"
#include "Topology.h"
#include "Route.h"

class CSolution
{
public:
	CSolution(void);
	CSolution(const CSolution&);
	~CSolution(void);
	CSolution& operator=(const CSolution&);

/*Attributes*/	
	LISTROUTE routelist;//the queue of routes in solution;
	unsigned int sensor_num;//the total number of  mobile sensors in solution
	unsigned int route_num;// the total number of routes in solution
	double totallength;
	int **pRouteMatrix;//RouteMatrix，如果POI之间有边，则设置为1，否则为0
	double **pContactRate;//ProbabilityMatrix, working with routematrix,表示边的概率。

/*Open Interfaces*/
	//邻域解产生,由当前解的副本调用，将该副本转化为一个邻域解。其中可调用多个邻域解产生方法
	virtual void NeighborGenerate();
	//由一个只由构造函数产生的空解调用，将该空解转化为初始解。其中可调用多个初始解产生方法
	virtual void Initialization(); //初始解产生；
	//对当前解进行局部优化。其中可调用多个局部优化方法。
	virtual void LocalOptimize();

/*Supportive functions used by 构造函数*/
	//为pRouteMatrix和pContactMatrix分配内存
	void InitialRouteMatrix();
	//为routlist分配内存
	void InitialRouteList();

/*
Important Functions
*/

	//根据routelist更新相应的属性.赋值sensor_num,route_num,totalllength.
	void UpdateStatus();
	//根据pRouteMatrix生成当前实例的routelist
	void RouteMatrix_to_List();
	//根据routelist生成当前实例的pRouteMatrix
	void List_to_RouteMatrix();
	//打印当前解
	void PrintSolution(void);


/*初始解产生方法*/
	/*为当前解产生一个随机初始解
	1.产生一个随机pContactMatrix
	2.产生相应的pRouteMatrix
	3 产生相应的routelist
	4.Update 当前解的各项属性
	*/ 
	void GetRandomSolution();
	/*产生一个随机pContactMatrix，由GetRandomSolution在调用RoutingBuid之前调用*/
	void GetRandomContactRate();
	/*根据pContactMatrix，构建pRouteMatrix*/
	void RoutingBuild();
	//距离越近，选中概率越大的方式产生pContactMatrix
	void GetDisMinFirstContactRate();
	//基于公式：if　tij-Lij/ v＞0则Pij=（tij-Lij/ v）／ｔｉｊ,产生pContactMatrix
	void GetByTLContactRate();


/*邻域解产生算法*/
	//任选一条路径，在该路径中任选一POI删除，再任选一路径，将删除的POI重新插入。注意必须调用UpdateStatus
	void Random_Delete_One_POI();
	//选择多个不同POI从原来的路径删除，然后依次插入任意路径
	void Random_Delete_Multiple_POI(int m);
	//在路径覆盖效率小于解覆盖效率的路径中任选一条路径，随机删除一个POI，插入任意一条路径。
	void Random_Delete_POI_By_CoverageRate();
	//一点交换，选中两条路径，再在各自路径中选中一条边，断开，交叉链接，生成两条新路径
	void One_Point_Crossover();
	//两点交换，选中两条路径，再在各自路径中选中一条边的两个点（共四个点），按一定规律分别划分到两条路径上，生成两条新路径
	void Two_Point_Crossover();
	//局部搜索，试图将那些点很少的路径中的点插入到其他路径中去，将这些短点路径消除
	void Additional_Local_Search();
	//基于覆盖效率选择POI，试探插入最短路径
	void Delete_POI_on_ContactRate();
	/*
	从当前实例中随机删除一个POI，返回删除的POI的ID和删除后该POI后的不完整解。
	不更改当前实例的内容。输入参数uncompletedsolution是当前实例的一个备份。
	*/
	unsigned int  RandomDeleteOnePOI(CSolution& uncompletedsolution);
	/*
	从当前实例中根据覆盖效率随机删除一个POI，返回删除的POI的ID和删除后该POI后的不完整解。
	不更改当前实例的内容。输入参数uncompletedsolution是当前实例的一个备份。
	*/
	unsigned int  RandomDeleteOnePOI_ContactRate(CSolution& uncompletedsolution);

	int GetPOINumber();
	
//	bool Insert_One_POI_Basedon_ContactRate(CSolution& uncompletedsolution,unsigned int poiid);
	bool Random_Insert_One_POI(CSolution& uncompletedsolution,unsigned int poiid);
	bool Insert_Best_Route(CSolution& uncompletedsolution,unsigned int poiid);
};

