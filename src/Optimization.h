#pragma once
#include "GlobalParameters.h"
#include "Topology.h"
#include "Solution.h"

class COptimization//优化策略，不同的优化算法从该类派生产生新的优化算法
{        
public:
	COptimization(void);
	virtual ~COptimization(void);
	unsigned int NumberofSolution;
	virtual void GenerateInit(void);//初始解产生
	CSolution *pSolution; //操作的初始解

	
	
	virtual CSolution Optimize(void);//优化函数;



};

