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
	//���ز�����=
	CGA_Solution& operator=(const CGA_Solution& it);

	////Q:�������������;��
	//int birth;
	//��Ӧ��
	double fitnessvalue;
	//δ�Ľ�����
	unsigned int NumberofUnImproved;
	//Q:�������������;��
	unsigned int localbest;

	//�ж����������Ƿ���ͬ
	bool SameAs(/*const*/ CGA_Solution& it);

	//�ж�һ��POI�Ƿ��ѱ���������·���У�����true��false��route_indexΪPOIλ�ڵ�·���±�
	bool IfPOIExists(int poi_seq, int &route_index);

	//������Ӧ�Ȳ�����
	//ʹ��1/Z(x)��Ϊ��Ӧ�ȵ���ֵ
	//Z(x)= RATE_POI_NUM * sensor_num + RATE_DISTANCE * totallength
	double getFitnessValue();
	void setFitnessValue(double fitness);

	//�жϺϷ��ԣ�ֻҪ����·������Ϊ�վ���Ϊ�ǺϷ���
	bool IsValid();

	//�ڽ��·������ɾ��ָ����POI
	bool DeletePOI(int seq);
	//�ڽ��·������ɾ��ָ����POI��
	bool DeletePOIList(LISTINT seqlist);
	//�ٲ���ָ����POI��
	bool ReInsertPOIList(LISTINT poi_list);
	//ɾ���ظ��㣬����·���е�POI�ٲ��뵽�������Ľ���
	bool ReInsertRouteToSolution(BASIC_ROUTE_Struct route);

	//���������⼰����ÿ��·������Ratio = POI_num/sensor_num
	//RatioԽ�͵ĵ㱻ѡ�еĸ���Խ��
	void SelectRouteByRatio(int &route_n);  //ѡ��һ��·��
	void SelectRouteByRatio(int &route1_n, int &route2_n);   //ѡ������·��

	void UpdateStatus();
	void PrintSolution();

};

