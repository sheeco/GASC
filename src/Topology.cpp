#include "Topology.h"


CTopology::CTopology(void)
{
	pDistanceMatrix=NULL;
	itotalPOINumber=0;
	listPOI.clear();
}

CTopology::CTopology(const CTopology &it)
{
	//cout<<"yumen2"<<endl;
	this->itotalPOINumber = it.itotalPOINumber;
	listPOI=it.listPOI;
	pDistanceMatrix=new double*[itotalPOINumber];
	for(int i=0;i<itotalPOINumber;i++)
	{
		pDistanceMatrix[i]=new double[itotalPOINumber];
	}

	for(int i=0;i<itotalPOINumber;i++)
		for(int j=0;j<itotalPOINumber;j++)
		{
			pDistanceMatrix[i][j] = it.pDistanceMatrix[i][j];
		}
}

CTopology::CTopology(int TotalPOINumber)
{
	
	itotalPOINumber=TotalPOINumber;
	pDistanceMatrix=new double*[itotalPOINumber];
	for(int i=0;i<itotalPOINumber;i++)
	{
		pDistanceMatrix[i]=new double[itotalPOINumber];
	}
}

CTopology::~CTopology(void)
{
	//cout<<"CTopology Îö¹¹"<<endl;
	if (pDistanceMatrix!=NULL)
	{
		for(int i=0;i<itotalPOINumber;i++)
		{
			delete[] pDistanceMatrix[i];
		}
	}
	delete[] pDistanceMatrix;
	pDistanceMatrix=NULL;
	itotalPOINumber=0;
	listPOI.clear();
}

double CTopology::GetDistance(int iPoiNumber_i,int iPoiNumber_j)
{
	return pDistanceMatrix[iPoiNumber_i][iPoiNumber_j];
}

POI_Struct CTopology::get_single_POI(int index)
{
	LISTPOI::iterator i;
	POI_Struct poi;
	for (i= listPOI.begin(); i!= listPOI.end(); ++i)
	{
		if((*i).id==index){
			memcpy(&poi,&(*i),sizeof(POI_Struct));
			break;
		}
	}
	return poi;
}



void CTopology::ClearDistanceMatrix(void)
{

	if (pDistanceMatrix!=NULL)
	{
		for(int i=0;i<itotalPOINumber;i++)
		{
			delete[] pDistanceMatrix[i];
		}
	}
	delete[] pDistanceMatrix;
	pDistanceMatrix=NULL;
	itotalPOINumber=0;
	listPOI.clear();
}

CTopology& CTopology::operator=(CTopology &it)
{
	//cout<<"yumen"<<endl;
	this->itotalPOINumber = it.itotalPOINumber;
	if(this->pDistanceMatrix != NULL)
	{
		ClearDistanceMatrix();
	}
	listPOI=it.listPOI;
	pDistanceMatrix=new double*[itotalPOINumber];
	for(int i=0;i<itotalPOINumber;i++)
	{
		pDistanceMatrix[i]=new double[itotalPOINumber];
	}

	for(int i=0;i<itotalPOINumber;i++)
		for(int j=0;j<itotalPOINumber;j++)
		{
			pDistanceMatrix[i][j] = it.pDistanceMatrix[i][j];
		}

	return *this;
}
