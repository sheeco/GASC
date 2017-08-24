#pragma once
#include "solution.h"
#include <vector>

extern double RATE_SENSOR_NUM;
extern double RATE_DISTANCE;
extern ofstream savetofile;

class CGA_Solution :
	public CSolution
{
public:
	CGA_Solution(void);
	CGA_Solution(const CSolution& it);
	~CGA_Solution(void);
	//重载操作符=
	CGA_Solution& operator=(const CGA_Solution& it);

	////Q:变量的意义和用途？
	//int birth;
	//适应度
	double fitnessvalue;
	//未改进次数
	unsigned int NumberofUnImproved;
	//Q:变量的意义和用途？
	unsigned int localbest;

	//判断两个个体是否相同
	bool SameAs(/*const*/ CGA_Solution& it);

	//判断一个POI是否已被包含在了路径中，返回true或false，route_index为POI位于的路径下标
	bool IfPOIExists(int poi_seq, int &route_index);

	//计算适应度并返回
	//使用1/Z(x)作为适应度的数值
	//Z(x)= RATE_POI_NUM * sensor_num + RATE_DISTANCE * totallength
	double getFitnessValue();
	void setFitnessValue(double fitness);

	//判断合法性，只要所有路径都不为空就认为是合法的
	bool IsValid();

	//在解的路径集中删除指定的POI
	bool DeletePOI(int seq);
	//在解的路径集中删除指定的POI集
	bool DeletePOIList(LISTINT seqlist);
	//再插入指定的POI集
	bool ReInsertPOIList(LISTINT poi_list);
	//删除重复点，并将路径中的POI再插入到不完整的解中
	bool ReInsertRouteToSolution(BASIC_ROUTE_Struct route);

	//对于整个解及其中每条路径都有Ratio = POI_num/sensor_num
	//Ratio越低的点被选中的概率越大
	void SelectRouteByRatio(int &route_n);  //选择一条路径
	void SelectRouteByRatio(int &route1_n, int &route2_n);   //选择两条路径

	void UpdateStatus();
	void PrintSolution();

};

