#pragma once
#include "GlobalParameters.h"
#include "Topology.h"
#include "Solution.h"

class COptimization//�Ż����ԣ���ͬ���Ż��㷨�Ӹ������������µ��Ż��㷨
{        
public:
	COptimization(void);
	virtual ~COptimization(void);
	unsigned int NumberofSolution;
	virtual void GenerateInit(void);//��ʼ�����
	CSolution *pSolution; //�����ĳ�ʼ��

	
	
	virtual CSolution Optimize(void);//�Ż�����;



};

