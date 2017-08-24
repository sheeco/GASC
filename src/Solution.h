#pragma once
#include "GlobalParameters.h"
#include "Topology.h"
#include "Route.h"

class CSolution
{
public:
	CSolution(void);
	CSolution(const CSolution&);
	~CSolution(void);
	CSolution& operator=(const CSolution&);

/*Attributes*/	
	LISTROUTE routelist;//the queue of routes in solution;
	unsigned int sensor_num;//the total number of  mobile sensors in solution
	unsigned int route_num;// the total number of routes in solution
	double totallength;
	int **pRouteMatrix;//RouteMatrix�����POI֮���бߣ�������Ϊ1������Ϊ0
	double **pContactRate;//ProbabilityMatrix, working with routematrix,��ʾ�ߵĸ��ʡ�

/*Open Interfaces*/
	//��������,�ɵ�ǰ��ĸ������ã����ø���ת��Ϊһ������⡣���пɵ��ö��������������
	virtual void NeighborGenerate();
	//��һ��ֻ�ɹ��캯�������Ŀս���ã����ÿս�ת��Ϊ��ʼ�⡣���пɵ��ö����ʼ���������
	virtual void Initialization(); //��ʼ�������
	//�Ե�ǰ����оֲ��Ż������пɵ��ö���ֲ��Ż�������
	virtual void LocalOptimize();

/*Supportive functions used by ���캯��*/
	//ΪpRouteMatrix��pContactMatrix�����ڴ�
	void InitialRouteMatrix();
	//Ϊroutlist�����ڴ�
	void InitialRouteList();

/*
Important Functions
*/

	//����routelist������Ӧ������.��ֵsensor_num,route_num,totalllength.
	void UpdateStatus();
	//����pRouteMatrix���ɵ�ǰʵ����routelist
	void RouteMatrix_to_List();
	//����routelist���ɵ�ǰʵ����pRouteMatrix
	void List_to_RouteMatrix();
	//��ӡ��ǰ��
	void PrintSolution(void);


/*��ʼ���������*/
	/*Ϊ��ǰ�����һ�������ʼ��
	1.����һ�����pContactMatrix
	2.������Ӧ��pRouteMatrix
	3 ������Ӧ��routelist
	4.Update ��ǰ��ĸ�������
	*/ 
	void GetRandomSolution();
	/*����һ�����pContactMatrix����GetRandomSolution�ڵ���RoutingBuid֮ǰ����*/
	void GetRandomContactRate();
	/*����pContactMatrix������pRouteMatrix*/
	void RoutingBuild();
	//����Խ����ѡ�и���Խ��ķ�ʽ����pContactMatrix
	void GetDisMinFirstContactRate();
	//���ڹ�ʽ��if��tij-Lij/ v��0��Pij=��tij-Lij/ v���������,����pContactMatrix
	void GetByTLContactRate();


/*���������㷨*/
	//��ѡһ��·�����ڸ�·������ѡһPOIɾ��������ѡһ·������ɾ����POI���²��롣ע��������UpdateStatus
	void Random_Delete_One_POI();
	//ѡ������ͬPOI��ԭ����·��ɾ����Ȼ�����β�������·��
	void Random_Delete_Multiple_POI(int m);
	//��·������Ч��С�ڽ⸲��Ч�ʵ�·������ѡһ��·�������ɾ��һ��POI����������һ��·����
	void Random_Delete_POI_By_CoverageRate();
	//һ�㽻����ѡ������·�������ڸ���·����ѡ��һ���ߣ��Ͽ����������ӣ�����������·��
	void One_Point_Crossover();
	//���㽻����ѡ������·�������ڸ���·����ѡ��һ���ߵ������㣨���ĸ��㣩����һ�����ɷֱ𻮷ֵ�����·���ϣ�����������·��
	void Two_Point_Crossover();
	//�ֲ���������ͼ����Щ����ٵ�·���еĵ���뵽����·����ȥ������Щ�̵�·������
	void Additional_Local_Search();
	//���ڸ���Ч��ѡ��POI����̽�������·��
	void Delete_POI_on_ContactRate();
	/*
	�ӵ�ǰʵ�������ɾ��һ��POI������ɾ����POI��ID��ɾ�����POI��Ĳ������⡣
	�����ĵ�ǰʵ�������ݡ��������uncompletedsolution�ǵ�ǰʵ����һ�����ݡ�
	*/
	unsigned int  RandomDeleteOnePOI(CSolution& uncompletedsolution);
	/*
	�ӵ�ǰʵ���и��ݸ���Ч�����ɾ��һ��POI������ɾ����POI��ID��ɾ�����POI��Ĳ������⡣
	�����ĵ�ǰʵ�������ݡ��������uncompletedsolution�ǵ�ǰʵ����һ�����ݡ�
	*/
	unsigned int  RandomDeleteOnePOI_ContactRate(CSolution& uncompletedsolution);

	int GetPOINumber();
	
//	bool Insert_One_POI_Basedon_ContactRate(CSolution& uncompletedsolution,unsigned int poiid);
	bool Random_Insert_One_POI(CSolution& uncompletedsolution,unsigned int poiid);
	bool Insert_Best_Route(CSolution& uncompletedsolution,unsigned int poiid);
};

