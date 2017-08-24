#pragma once
#include "GA_Solution.h"
#include "optimization.h"
#include <vector>

extern int POPULATION_SIZE;
extern int MAX_UNIMPROVED_NUM;
extern int MAX_SOLUTION_NUM;
extern double PROB_CROSSOVER;  //交叉概率
extern double PROB_MUTATION;  //变异概率
extern ofstream savetofile;

typedef vector<CGA_Solution> LISTSOLUTION;
extern CTopology topo;

class CGA :
	public COptimization
{
public:
	CGA(void);
	~CGA(void);

	CGA_Solution BestSolution;
	LISTSOLUTION pSolution;  //操作的初始解
	int NumberofSolution;
	int NumberofUnImproved;  //局部最优解没有改善的计数
	int BestUnImproved;  //最优解没有改善的计数

	//double buffer;
	int current_op1;
	int current_op2;
	//int inumCrossover;
	//int inumMutation;
	//int inumEffectiveCrossover;
	//int inumEffectiveMutation;
	int runninground;
	int worst_first;
	int worst_second;
	double sumFitnessValue;
	//int seed;

	//初始解产生，由main函数调用
	void GenerateInit(void);

	//优化函数，控制主要过程，由main函数调用
	CSolution Optimize(void);

	/*几种交叉、变异算子*/
	//对称匹配交叉
	bool SymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2);
	//非对称匹配交叉
	bool UnSymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2);
	//线路交叉
	bool RouteCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2);
	//线路变异(R1算子)
	bool RouteMutution(CGA_Solution &indiv1, CGA_Solution &output1);
	//单点变异(R2算子)
	bool POIMutution(CGA_Solution &indiv1, CGA_Solution &output1);

	/*Supportive Functions*/
	//按轮盘赌选择两个个体，并将 <var>current_op1</var> 和 <var>current_op2</var> 指向它们
	//如果成功，返回true
	bool SelectTwo();

	//找到当前种群中适应度最好的个体，存入BestSolution；计算适应度的总和
	//函数更新BestUnImproved，在每次迭代过程的最后调用一次，注意不可重复调用
	void FindBestSolution();

	//找到当前种群中适应度最差的两个个体用于替换
	//将<var>worst_first</var>和<var>worst_second</var>指向它们
	void FindWorstSolution();

	//判断一个个体是否与当前种群中的个体相同
	bool IfExists(CGA_Solution& it);

	//判断是否符合结束条件，即达到MAX_UNIMPROVED_NUM和MAX_SOLUTION_NUM
	bool OptCompleted();

	void finalstatus();
};

