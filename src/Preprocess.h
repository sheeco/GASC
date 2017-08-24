#pragma once
/*
读取TOPOLOGY文件，产生一个POI距离矩阵的实例
*/
#include "GlobalParameters.h"
#include "Topology.h"
class Preprocess
{
public:
	//暂时可以不用，当要随机产生topo时实现
	Preprocess(void);
	~Preprocess(void);

	/*根据指定的topo文件，读取内容存入到listPOI中,*/
	Preprocess(string filename);

	LISTPOI listPOI;//POI list(P_node)
	
	/*被Preprocess(filename)调用，用于文件读取操作*/
	bool ReadToplogyfile(string filename);//读取topology文件，存入listPOI中。我觉得把这个函数可以放在这里
	


	/*
	GenerateSpecialTopology()输入用于产生一个特定的topology,而不是使用一个topo文件，目前不需要实现,如果需要使用，可在构造函数Preprocess()中调用
	
	void GenerateSpecialTopology(int poi_num);
	*/

	// To generate a instance of CTopology
	CTopology GenerateCTopology(void);
	
	/* To calculate the distance between Pi and Pj,based on the listPOI. This function is called by GenerateCTopology*/
	double CalculateDistance(int iPoiNumber_i,int iPoiNumber_j);

	
	
};

