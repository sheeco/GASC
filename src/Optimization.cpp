#include "Optimization.h"

extern CTopology topo;

COptimization::COptimization()
{
	NumberofSolution=0;
	pSolution=NULL;
}


COptimization::~COptimization(void)
{
	NumberofSolution=0;
	cout<<"pSolution¾ØÕóÎö¹¹"<<endl;
	if(pSolution!=NULL)
	{
		delete[]pSolution;
	}
	pSolution=NULL;
}


void COptimization::GenerateInit(void)
{
	
}


CSolution COptimization::Optimize(void)
{
	return CSolution();
}
