#include "Preprocess.h"
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
extern CTopology topo;


Preprocess::Preprocess(void)
{
	listPOI.clear();
}


Preprocess::~Preprocess(void)
{
	listPOI.clear();
}



Preprocess::Preprocess(string filename)
{
	listPOI.clear();
	ReadToplogyfile(filename);
}

bool Preprocess::ReadToplogyfile(string filename)
{
	bool bl=true;
	FILE* fp;
	POI_Struct node1;
	fopen_s(&fp,filename.c_str(),"r");
	//因为topo文件的格式和现在的node不统一，所以加两个变量来统一格式
	double r_time,s_time;
	while(!feof(fp))
	{
		fscanf_s(fp,"%d",&node1.id);    //read poi id
		fscanf_s(fp,"%d",&node1.x_coor); 
		fscanf_s(fp,"%d",&node1.y_coor);
		fscanf_s(fp,"%lf",&node1.demand);
		fscanf_s(fp,"%d",&r_time);
		fscanf_s(fp,"%d",&node1.d_time);
		fscanf_s(fp,"%d",&s_time);
		//appendPOI(node1);
		listPOI.push_back(node1);
	}
	fclose(fp);
	return bl;
}


CTopology Preprocess::GenerateCTopology(void)
{
	int itotalPOINumber=listPOI.size();
	CTopology ff=CTopology(itotalPOINumber);
	
	ff.pDistanceMatrix=new double*[itotalPOINumber];
	for(int i=0;i<itotalPOINumber;i++)
	{
		ff.pDistanceMatrix[i]=new double[itotalPOINumber];
	}
	LISTPOI::iterator j,k;
	int m=0,n=0;
	ff.listPOI=listPOI;
	if(!listPOI.empty())
	{
		for(j=listPOI.begin();j!=listPOI.end();j++)
		{
			n=0;
			for(k=listPOI.begin();k!=listPOI.end();k++)
			{
				ff.pDistanceMatrix[m][n]=sqrt(pow((double)((*j).x_coor-(*k).x_coor),2)+pow((double)((*j).y_coor-(*k).y_coor),2));
				n++;
			}
			m++;
		}
	}
	else
	{
		cout<<"error:GenerateCTopology() listPOI is empty!"<<endl;
	}
	return ff;// topo;
}
