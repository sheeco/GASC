#pragma once
#include "GlobalParameters.h"
#include "Topology.h"

/*
����·��ʱ��һЩע��������Ҫ����InsertAfter()��������õ�ǰ·������һ����׼
״̬������sink��ͷ��sink��β������ΪInsertAfter()��ÿ�β���֮������·����Ϣ��
���Ҫ��·��������0��ͷ��0��β�Ľṹ�������п��ܻ����
*/


class CRoute
{
public:
	CRoute(void);
	~CRoute(void);
	CRoute(BASIC_ROUTE_Struct route);
	
	/*The Only Attribute*/
	BASIC_ROUTE_Struct current_route;

	/*·�������Լ��飺����һ��POI��id��Ϊ���������齫POI����·���Ƿ���� */
	//ֻ��鵱ǰ·����capacity�Ƿ��������POI��demand
	bool IsFeasible(int poi_id);//�����Ƿ���У������buffer��,����������Ը��ݲ�ͬ��Ҫ���д��У�����
	bool IsFeasibleTimeAndBuffer(int poi_id);//�����Ƿ���У����buffer��ʱ�䣩

	/*����current_route.seq����current_route��������Ա��������current_route�ķ������У�����·���ܳ��ȣ�·����POI������������sink����·������ڵ�������·��capacity*/
	void UpdateStatus();

	/*��ӡ��ǰ·��*/
	void PrintRoute();

	/*���·�����Ż��������ɲ������*/
	//2opt
	BASIC_ROUTE_Struct Two_Opt(BASIC_ROUTE_Struct*brs);

	/*·����صĲ�����ɾ���������ڲ������ǰ�������IsFeasible������ܷ���в��룬�����в�������ǰ�������updatestatus*/
	/*
	called by create_Initializeial_solution and may be called by Neighbor();
	insert p(id) into p(pos).This function will update current_route;
	*/
	/*
	����·��ʱ��һЩע��������Ҫ����InsertAfter()��������õ�ǰ·������һ����׼
	״̬������sink��ͷ��sink��β������ΪInsertAfter()��ÿ�β���֮������·����Ϣ��
	���Ҫ��·��������sink��ͷ��sink��β�Ľṹ�������п��ܻ����
	*/
	bool InsertAfter(int pos,int id);//����ǰ�ڵ��Ϊid�Ľڵ㣬�����ڽڵ��Ϊpos��POI�ڵ�֮��

	/*
	maybe called by Neighbor()
	This function will update current_route;
	*/
	bool DeleteFromRoute(int id);//ɾ���ڵ��Ϊid��POI�ڵ�

	bool SolomenInsert(int id);//�ҵ�ǰ·����С�����ĵط�������Ϊid�Ľڵ�

	bool Is_Empty();//��ǰ·���Ƿ���ֻ������sink�ڵ������������Ϣ
};