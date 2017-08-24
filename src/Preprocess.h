#pragma once
/*
��ȡTOPOLOGY�ļ�������һ��POI��������ʵ��
*/
#include "GlobalParameters.h"
#include "Topology.h"
class Preprocess
{
public:
	//��ʱ���Բ��ã���Ҫ�������topoʱʵ��
	Preprocess(void);
	~Preprocess(void);

	/*����ָ����topo�ļ�����ȡ���ݴ��뵽listPOI��,*/
	Preprocess(string filename);

	LISTPOI listPOI;//POI list(P_node)
	
	/*��Preprocess(filename)���ã������ļ���ȡ����*/
	bool ReadToplogyfile(string filename);//��ȡtopology�ļ�������listPOI�С��Ҿ��ð�����������Է�������
	


	/*
	GenerateSpecialTopology()�������ڲ���һ���ض���topology,������ʹ��һ��topo�ļ���Ŀǰ����Ҫʵ��,�����Ҫʹ�ã����ڹ��캯��Preprocess()�е���
	
	void GenerateSpecialTopology(int poi_num);
	*/

	// To generate a instance of CTopology
	CTopology GenerateCTopology(void);
	
	/* To calculate the distance between Pi and Pj,based on the listPOI. This function is called by GenerateCTopology*/
	double CalculateDistance(int iPoiNumber_i,int iPoiNumber_j);

	
	
};

