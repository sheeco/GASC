#pragma once
#include "GlobalParameters.h"

class CTopology
{
public:
	//Never used
	CTopology(void);
	CTopology(const CTopology &);
	~CTopology(void);
	//Called by Preprocess::GenerateCTopology;Input para TotalPOINumber includes the sink nodes 
	CTopology(int TotalPOINumber);
	//initialized in CTopology(TotalPOINumber),evalued in Preprocess::GenerateCTopology 
	int itotalPOINumber;//poiNumber
	double **pDistanceMatrix;//æ‡¿Îæÿ’Û,
	LISTPOI listPOI;//POI list(include P_node)
	double GetDistance(int iPoiNumber_i,int iPoiNumber_j);//return the distance of POI(NO. i)and(NO.j)
	POI_Struct get_single_POI(int index);
	void ClearDistanceMatrix(void);

	CTopology& operator=(CTopology&);
};

