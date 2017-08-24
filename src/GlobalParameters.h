#pragma once

// TODO: 在此处引用程序需要的其他头文件
#include <iostream>
#include <fstream>
#include <list>
#include <cmath>
#include <algorithm>
#include <string.h>
#include <string>
#include <iomanip>
#include <stdlib.h>
#include <time.h>


using namespace std;

//从main函数输入的速度和buffer参数
extern unsigned int BUFF_MAX;
extern unsigned int M_VELOCITY;
//extern CTopology topo;

//struct for each POI
typedef struct{
	int id;//
	int x_coor;//x coordinate
	int y_coor;//y coordinate
	double demand;//the max data generated in the service time
	int d_time;//time interval
}POI_Struct;//Data structure

//struct for each mobile sensor node
typedef struct{
	double buffer;
	double bandwidth;//preserved
	double velocity;
}SENSOR_Struct;//data structure:sensor

typedef list<int> LISTINT;  
typedef list<POI_Struct> LISTPOI;
typedef list<SENSOR_Struct> LISTSENSOR;

//basic struct for each route,users may extend specific defInitializeion for each agorithm based on this struct.
typedef struct{
	double capacity;//the current capacity of this route that can't be larger than one single sensor's buffersize 
	double len; //the length of this route
	int poi_num;//the number of POIs that are assigned to this route;
	int sensor_num; //the number of mobile sensors that are assigned to this route;
	LISTINT seq; //the POI access sequence which will be used to index the LISTPOI;
}BASIC_ROUTE_Struct;// the data structure of route


typedef list<BASIC_ROUTE_Struct> LISTROUTE; 

typedef struct{
	LISTROUTE routelist;//the queue of routes in solution;
	unsigned int sensor_num;//the total number of  mobile sensors in solution
	unsigned int route_num;// the total number of routes in solution
}SOLUTION_Struct; // the data structure of solution


		/*Randomly product a fload number between min and max*/
	inline double RandomFloat(double min,double max)
	{
		if(min==max)
			return min;
		if(min > max)
		{
			double temp = max;
			max = min;
			min = temp;
		}
		//srand( (unsigned)time(NULL));
		return min + (double)rand() / RAND_MAX * (max - min);
	}
	inline int RandomInt(int min, int max)
	{
		if (min==max)
			return min;
		if(min > max)
		{
			int temp = max;
			max = min;
			min = temp;
		}

		//srand( (unsigned)time(NULL)); 
		return min + rand()%(max-min);
	}
 
	/*----------一点交换，两点交换算法会用到的数据结构-----------*/

typedef struct{
int sensor_num;//两条路径的移动节点总数
double all_len;//两条路径的总长度
BASIC_ROUTE_Struct mm;
BASIC_ROUTE_Struct nn;
}Asetofpaths;





