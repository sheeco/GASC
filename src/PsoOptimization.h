#pragma once
#include "optimization.h"
#define Particle_Num 100	//粒子群中粒子个数
#define outer 100	//外循环次数
//#define Weight 0.7	//惯性指数
#define Coefﬁcients 2	//相关系数，从论文例子里面的看见的
#define Pc 0	//更新公式中，选择Pbest时的阀值
class PsoOptimization :
	public COptimization
{
public:
	PsoOptimization(void);
	~PsoOptimization(void);

	void GenerateInit(void);//初始解产生
	CSolution Optimize(void);//优化函数;

	/*pso支持代码*/

	
	LISTPOI temPOI;//用于路径生成算法Route_Building();while的结束条件即temPOI为空：生成的solution将所有的节点都访问到了。
	double ***p;//p矩阵：每个粒子有一个自己的P矩阵（路径生成过程中每条边选中的概率）
	double Weight;

	CSolution *pBestSolution;//局部最优解
	CSolution gBestSolution;//全局最优解

	void initP(int Particle_No);//基于最短距离的初始化p矩阵
	void initPbytl(int Particle_No);//if　tij-Lij/ v＞0则Pij=（tij-Lij/ v）／ｔｉｊ
	void rdinitP(int Particle_No);//随机初始化目标矩阵P

	LISTINT Generate_Candidate_POI(int NodeId_t,BASIC_ROUTE_Struct current_route,int Particle_No);//利用p矩阵和当前节点产生下一个候选节点
	CSolution Route_Building(int Particle_No);//利用候选节点生成路径
	
	bool checkId(int NodeId);//检查NodeId是否在temPOI中，若在，则返回true，不在，则返回false	 called by Generate_Candidate_POI(int current_POI_id)
	void removePOI(int NodeId);//从temPOI中删除指定id的POI called by Route_Building()
	bool Check_Buffer(BASIC_ROUTE_Struct srs);//检查目标路径是否满足buffer要求

	
	void updateGbest();//更新全局最优解 called by GenerateInit(); Optimize(); 
	void updatePbest();//更新局部最优解 called by GenerateInit(); Optimize();

	double reComputeAllLength(CSolution);//计算solution的总长度
};

