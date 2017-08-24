#pragma once
#include "GA_Solution.h"
#include "optimization.h"
#include <vector>

extern int POPULATION_SIZE;
extern int MAX_UNIMPROVED_NUM;
extern int MAX_SOLUTION_NUM;
extern double PROB_CROSSOVER;  //�������
extern double PROB_MUTATION;  //�������
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
	LISTSOLUTION pSolution;  //�����ĳ�ʼ��
	int NumberofSolution;
	int NumberofUnImproved;  //�ֲ����Ž�û�и��Ƶļ���
	int BestUnImproved;  //���Ž�û�и��Ƶļ���

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

	//��ʼ���������main��������
	void GenerateInit(void);

	//�Ż�������������Ҫ���̣���main��������
	CSolution Optimize(void);

	/*���ֽ��桢��������*/
	//�Գ�ƥ�佻��
	bool SymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2);
	//�ǶԳ�ƥ�佻��
	bool UnSymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2);
	//��·����
	bool RouteCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2);
	//��·����(R1����)
	bool RouteMutution(CGA_Solution &indiv1, CGA_Solution &output1);
	//�������(R2����)
	bool POIMutution(CGA_Solution &indiv1, CGA_Solution &output1);

	/*Supportive Functions*/
	//�����̶�ѡ���������壬���� <var>current_op1</var> �� <var>current_op2</var> ָ������
	//����ɹ�������true
	bool SelectTwo();

	//�ҵ���ǰ��Ⱥ����Ӧ����õĸ��壬����BestSolution��������Ӧ�ȵ��ܺ�
	//��������BestUnImproved����ÿ�ε������̵�������һ�Σ�ע�ⲻ���ظ�����
	void FindBestSolution();

	//�ҵ���ǰ��Ⱥ����Ӧ�������������������滻
	//��<var>worst_first</var>��<var>worst_second</var>ָ������
	void FindWorstSolution();

	//�ж�һ�������Ƿ��뵱ǰ��Ⱥ�еĸ�����ͬ
	bool IfExists(CGA_Solution& it);

	//�ж��Ƿ���Ͻ������������ﵽMAX_UNIMPROVED_NUM��MAX_SOLUTION_NUM
	bool OptCompleted();

	void finalstatus();
};

