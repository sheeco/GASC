#include "Solution.h"
#include <vector>
extern CTopology topo;

CSolution::CSolution(void)
{
	routelist.clear();
	sensor_num=0;
	route_num=0;
	totallength=0;
	pRouteMatrix=NULL;
	pContactRate=NULL;
	InitialRouteMatrix();
	InitialRouteList();
}

CSolution::CSolution(const CSolution& it)
{
	//cout<<"CSolution ��������"<<endl;
	int i,j;
	this->routelist=it.routelist;
	this->route_num=it.route_num;
	this->sensor_num=it.sensor_num;
	this->totallength=it.totallength;
	//if(pRouteMatrix!=NULL)
	//{
	//	ClearRouteMatrix();
	//}
	pRouteMatrix=new int*[topo.itotalPOINumber];
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		pRouteMatrix[i]=new int[topo.itotalPOINumber];
	}
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			this->pRouteMatrix[i][j]=it.pRouteMatrix[i][j];
		}
	}
	pContactRate=new double*[topo.itotalPOINumber];
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		pContactRate[i]=new double[topo.itotalPOINumber];
	}
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			this->pContactRate[i][j]=it.pContactRate[i][j];
		}
	}
}

CSolution& CSolution::operator=(const CSolution& it)
{
	//cout<<"operator= �� ���� "<<endl;
	int i,j;
	this->routelist=it.routelist;
	this->route_num=it.route_num;
	this->sensor_num=it.sensor_num;
	this->totallength=it.totallength;
	//pRouteMatrix=new int*[topo.itotalPOINumber];
	//for(i=0;i<topo.itotalPOINumber;i++)
	//{
	//	pRouteMatrix[i]=new int[topo.itotalPOINumber];
	//}
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			this->pRouteMatrix[i][j]=it.pRouteMatrix[i][j];
		}
	}
	//pContactRate=new double*[topo.itotalPOINumber];
	//for(i=0;i<topo.itotalPOINumber;i++)
	//{
	//	pContactRate[i]=new double[topo.itotalPOINumber];
	//}
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			this->pContactRate[i][j]=it.pContactRate[i][j];
		}
	}
	return *this;

}


CSolution::~CSolution(void)
{
	//cout<<"CSolution pRouteMatrix ����"<<endl;
	if(pRouteMatrix!=NULL)
	{
		for(int i=0;i<topo.itotalPOINumber;i++)
		{
			delete[] pRouteMatrix[i];
		}
	}
	delete[]pRouteMatrix;
	pRouteMatrix=NULL;
	if(pContactRate!=NULL)
	{
		for(int i=0;i<topo.itotalPOINumber;i++)
		{
			delete[] pContactRate[i];
		}
	}
	delete[]pContactRate;
	pContactRate=NULL;
}

void CSolution::RoutingBuild()
{
	int i,j,m;
	int *flag=new int[topo.itotalPOINumber];
	double temp;//�е�������
	int max_target;
	const int sinkid=0;
	CRoute curr=CRoute();
	bool routecompleted=false;
	bool allcompleted=false;
	routelist.clear();
	//FIXME: 
	for (i=0;i<topo.itotalPOINumber;i++)
	{
		flag[i]=0;
	}

	while (!allcompleted)
	{
		curr.current_route.seq.clear();
		curr.current_route.poi_num=0;
		curr.current_route.capacity=BUFF_MAX;
		i=sinkid;
		//curr.current_route.seq.insert(curr.current_route.seq.end(),sinkid);
		curr.current_route.seq.push_back(sinkid);
		curr.current_route.seq.push_back(sinkid);
		routecompleted=false;
		while (!routecompleted)
		{
			temp=0;
			//looking for a POI j that has the max contact probility to i and never be added in route
			for (j=0;j<topo.itotalPOINumber;j++)
			{
				if ((pContactRate[i][j]>=temp)&&(flag[j]==0))
				{
					temp=pContactRate[i][j];
					max_target=j;
				}			
			}
			if (i==max_target)
			{
				if(max_target==0)
				{
					for(m=1;m<topo.itotalPOINumber;m++)
					{
						allcompleted=true;
						if(flag[m]==0)
						{
							allcompleted=false;
							break;
						}
					}//end of the m loop
				}//end of the if max_target==0
				break;
			}// end of the i==max_target
			//curr.current_route.seq.insert(curr.current_route.seq.end(),max_target);
			if (max_target!=sinkid)
			{
				//curr.current_route.seq.insert(curr.current_route.seq.end(),sinkid);
				//curr.InsertAfter(max_target,sinkid);
				//curr.InsertAfter(max_target,sinkid);
				if(curr.InsertAfter(i,max_target))
				{
					this->pRouteMatrix[i][max_target]=1;
					i=max_target;
					flag[i]=1;
				}
				else
				{
					if(!curr.Is_Empty())
					{
						curr.UpdateStatus();
						routelist.push_back(curr.current_route);
						i=sinkid;
						routecompleted=true;
						curr.current_route.seq.clear();
					}
				}
			}
			else
			{
				routecompleted=true;
				curr.UpdateStatus();
				routelist.push_back(curr.current_route);
				curr.current_route.poi_num = curr.current_route.seq.size()-2;
				i=sinkid;
				routecompleted=true;
				curr.current_route.seq.clear();
				curr.current_route.poi_num=0;
				curr.current_route.capacity=BUFF_MAX;
			}
		}//end route completed
	}//end of all the search
	UpdateStatus();
	delete flag;
}

void CSolution::GetRandomSolution()
{
	GetRandomContactRate();
	//GetDisMinFirstContactRate();
	//GetByTLContactRate();
	RoutingBuild();
}

void CSolution::GetRandomContactRate()
{
	int k,j;
	double temp;
	for(k=0;k<topo.itotalPOINumber;k++)
	{
		temp=0;
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			if(k==j)
				this->pContactRate[k][j]=0;
			else
			{
				//srand((unsigned)time(NULL));
				pContactRate[k][j]=(double)rand()/(RAND_MAX);
			}
			temp+=pContactRate[k][j];
		}
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			pContactRate[k][j]=pContactRate[k][j]/temp;
		}
	}
}

void CSolution::GetDisMinFirstContactRate()
{
	int k=0,j=0;
	double *Max_len=new double[topo.itotalPOINumber];
	double *All_Len=new double[topo.itotalPOINumber];

	for(k=0;k<topo.itotalPOINumber;k++)
	{
		Max_len[k]=All_Len[k]=0;
	}

	for(k=0;k<topo.itotalPOINumber;k++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			if(k==j)
				pContactRate[k][j]=0;
			else
			{
				pContactRate[k][j]=0;
				if(Max_len[k]<topo.GetDistance(k,j))
					Max_len[k]=topo.GetDistance(k,j);
			}
			All_Len[k]+=topo.GetDistance(k,j);
		}
		All_Len[k]=Max_len[k]*(topo.itotalPOINumber-1)-All_Len[k];
	}

	for(k=0;k<topo.itotalPOINumber;k++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			if(k==j)
				pContactRate[k][j]=0;
			else
			{
				pContactRate[k][j]=(double)(Max_len[k]-topo.GetDistance(k,j))*10/All_Len[k]/10;
			}
		}
	}

	delete[]Max_len;
	delete[]All_Len;
}

void CSolution::GetByTLContactRate()
{
	int k,j;
	double Psum=0,Psum1=0,Tij;

	for(k=0;k<topo.itotalPOINumber;k++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			if(k==j)
				pContactRate[k][j]=0;
			else
			{
				//p[Particle_No][k][j]=(double)rand()/RAND_MAX;
				Tij=abs(topo.get_single_POI(k).d_time-topo.get_single_POI(j).d_time);
				Psum=Tij-topo.GetDistance(k,j)/M_VELOCITY;
				if(Psum>0)
					pContactRate[k][j]=(double)Psum/Tij;
				else
				{
					pContactRate[k][j]=0;
				}
			}
			Psum1+=pContactRate[k][j];
		}
		for(j=0;j<topo.itotalPOINumber;j++)//��һ��
			pContactRate[k][j]=pContactRate[k][j]/Psum1;	
	}
}

void CSolution::Initialization()
{
	//RoutingBuild();
}

void CSolution::NeighborGenerate()
{
	//	this->Random_Delete_One_POI();
	//	this->Two_Point_Crossover();
	//	this->Random_Delete_POI_By_CoverageRate();
	//this->PrintSolution();
	CSolution uncompeleted=CSolution(*this);
	//	unsigned int deletePOI=	RandomDeleteOnePOI(uncompeleted);
	unsigned int deletePOI=	RandomDeleteOnePOI_ContactRate(uncompeleted);
	//cout<<"the deletepoi is "<<deletePOI<<endl;
	//uncompeleted.PrintSolution();
	Random_Insert_One_POI(uncompeleted,deletePOI);
	//cout<<"-----------------------------------"<<endl;
	//uncompeleted.PrintSolution();
	//	system("pause");

}

void CSolution::LocalOptimize()
{
}

void CSolution::InitialRouteMatrix()
{
	int i,j;
	pRouteMatrix=new int*[topo.itotalPOINumber];
	for(i=0;i<topo.itotalPOINumber;i++)
		pRouteMatrix[i]=new int[topo.itotalPOINumber];
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			pRouteMatrix[i][j]=0;
		}
	}
	pContactRate=new double*[topo.itotalPOINumber];
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		pContactRate[i]=new double[topo.itotalPOINumber];
	}
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			pContactRate[i][j]=0;
		}
	}
}

void CSolution::InitialRouteList()
{
	routelist.clear();
}

void CSolution::UpdateStatus()
{
	double all_len=0;
	int s_num1=0;
	LISTROUTE::iterator i;
	for(i=routelist.begin();i!=routelist.end();i++)
	{
		all_len+=(*i).len;
		s_num1+=(*i).sensor_num;
	}
	sensor_num=s_num1;
	route_num=routelist.size();
	totallength=all_len;
}

void CSolution::PrintSolution(void)
{
	double all_len=0;
	int s_num1=0,poi_num=0;
	LISTROUTE::iterator i;
	for(i=routelist.begin();i!=routelist.end();i++)
	{
		LISTINT list_nid;
		LISTINT::iterator j;
		list_nid=(*i).seq;
		for(j=list_nid.begin();j!=list_nid.end();++j)		//˳������ڵ㡣������Ⱥ�һһ��Ӧ��
			cout<<*j<<" ";
		cout<<" the sensor number of this route is "<<(*i).sensor_num;
		cout<<endl;
		poi_num=poi_num+((*i).seq.size()-2);
		all_len+=(*i).len;
		s_num1+=(*i).sensor_num;
	}
	sensor_num=s_num1;
	route_num=routelist.size();
	cout<<"sol.route_num	"<<route_num<<endl;
	cout<<"sol.sensor_num	"<<sensor_num<<endl;
	cout<<"sol.length	"<<all_len<<endl;
	cout<<"sol.poi_num	"<<poi_num<<endl;
}


void CSolution::RouteMatrix_to_List()
{
	int i,j;
	int next_POI_id=1,current_POI_id=0;

	CRoute crt=CRoute();

	routelist.clear();
	sensor_num=0;
	route_num=0;
	if(pRouteMatrix!=NULL)
	{
		for(i=0;i<topo.itotalPOINumber;i++)
		{	
			crt.current_route.capacity=BUFF_MAX;
			crt.current_route.len=0;
			crt.current_route.poi_num=0;
			crt.current_route.sensor_num=0;
			crt.current_route.seq.clear();
			crt.current_route.seq.push_back(0);
			crt.current_route.seq.push_back(0);
			if(pRouteMatrix[0][i]==1)
			{
				crt.InsertAfter(0,i);
				current_POI_id=i;
				while (current_POI_id!=0)
				{
					for(j=0;j<topo.itotalPOINumber;j++)
					{
						if(pRouteMatrix[current_POI_id][j]==1)
						{
							crt.InsertAfter(current_POI_id,j);
							current_POI_id=j;
							break;
						}
					}
				}
				crt.UpdateStatus();
				routelist.push_back(crt.current_route);
				UpdateStatus();
			}
			else
			{
				continue;
			}
		}
	}
	else
	{
		cout<<"error:RouteMatrix_to_List() RouteMatrix is NULL!"<<endl;
	}
}

void CSolution::List_to_RouteMatrix()
{
	LISTROUTE::iterator i;
	LISTINT::iterator j,next_POI;
	int m,n;
	if(!routelist.empty())
	{
		for(m=0;m<topo.itotalPOINumber;m++)
		{
			for(n=0;n<topo.itotalPOINumber;n++)
			{
				pRouteMatrix[m][n]=0;
			}
		}
		for(i=routelist.begin();i!=routelist.end();i++)
		{
			for(j=(*i).seq.begin();j!=--(*i).seq.end();j++)
			{
				next_POI=j;
				next_POI++;
				pRouteMatrix[*j][*next_POI]=1;
			}
		}
	}
	else
	{
		cout<<"error:List_to_RouteMatrix() routelist is empty!"<<endl;
	}
}

void  CSolution::Random_Delete_One_POI()
{
	/*Randomly Select One Route pointed by <Var>iteratorDeletedRoute</Var>*/
	int routeNoInList = rand() % routelist.size();
	LISTROUTE::iterator iteratorDeletedRoute = routelist.begin();
	for(int i = 0;i < routeNoInList;i++)
	{
		iteratorDeletedRoute++;
	}
	CRoute deltedRoute(*iteratorDeletedRoute);//��Ҫɾ��һ��POI��·���ĸ���
	BASIC_ROUTE_Struct routeForDel = *iteratorDeletedRoute;
	/*Randomly Slecte One POI for deleting*/
	int poiNoInSeq = rand() % (deltedRoute.current_route.seq.size() - 2);
	int selectedPOI = 0;
	LISTINT::iterator itPOI = deltedRoute.current_route.seq.begin();
	for(int i = -1;i < poiNoInSeq;i++)
	{
		itPOI++;
	}
	selectedPOI = *itPOI;
	if(selectedPOI <= 0) 
	{//ѡ�в��Ϸ�POI
		cout<<"POI INVALID!\n";
		system("pause");
	}
	bool deleted = deltedRoute.DeleteFromRoute(selectedPOI);

	/*Randomly Slecte One Route to insert*/
	LISTROUTE::iterator iteratorInsertedRoute = routelist.begin();
	if(!deleted)
	{//ɾ��POIʧ��
		cout<<"DELETE POI FAILED!\n";
		system("pause");
	}
	else
	{//ɾ���ɹ�
		*iteratorDeletedRoute = deltedRoute.current_route;//����ɾ��POI��·��
		std::vector<LISTROUTE::iterator> pointers;
		for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
		{
			pointers.push_back(it);
		}
		//�������
		//bool sucInsert = false;
		//std::random_shuffle(pointers.begin(),pointers.end());
		//while(!pointers.empty())
		//{
		//	iteratorInsertedRoute = pointers.back();
		//	CRoute insertedRoute(*iteratorInsertedRoute);//��Ҫ����һ��POI��·���ĸ���
		//	bool inserted = insertedRoute.SolomenInsert(selectedPOI);
		//	if(!inserted)
		//	{//������ѡ���·�����ܲ���
		//		pointers.pop_back();
		//	}
		//	else
		//	{//����ɹ�������routelist
		//		if(deltedRoute.Is_Empty())
		//		{//���ɾ��POI��·��Ϊ�գ�ɾ����·��
		//			routelist.erase(iteratorDeletedRoute);
		//		}
		//		*iteratorInsertedRoute = insertedRoute.current_route;//���²���POI��·��
		//		sucInsert = true;
		//		break;
		//	}
		//}
		//if(!sucInsert)
		//{//�Ҳ����ܲ����·��
		//	*iteratorDeletedRoute = routeForDel;
		//}

		//ѡ����õ�λ�ò���
		bool sucInsert = false;
		int delta_sesor = 0;
		double delta_len = 0.0;
		int bestIndex = -1;//Ҫ����·�����±�
		CRoute *tempRouteList = new CRoute[pointers.size()];//��ѡ����·���б�
		for(unsigned int i = 0;i < pointers.size();i++)
		{
			tempRouteList[i].current_route = *pointers[i];
			bool inserted = tempRouteList[i].SolomenInsert(selectedPOI);
			bool better = false;
			if(inserted)
			{
				sucInsert = true;
				int temp_delta_sesor = tempRouteList[i].current_route.sensor_num - pointers[i]->sensor_num;
				double temp_delta_len = tempRouteList[i].current_route.len - pointers[i]->len;
				if(bestIndex == -1)
				{
					better = true;
				}
				else
				{
					if(temp_delta_sesor < delta_sesor)
					{//���sensor����С
						better = true;
					}
					if((temp_delta_sesor == delta_sesor) && (temp_delta_len < delta_len))
					{//���senrsor���䣬����·������С
						better =true;
					}
				}

				if(better)
				{
					bestIndex = i;
					delta_sesor = temp_delta_sesor;
					delta_len = temp_delta_len;
				}
			}
		}

		if(!sucInsert)
		{//�Ҳ����ܲ����·��
			*iteratorDeletedRoute = routeForDel;
		}
		else
		{//�ҵ���õĲ���·��
			*pointers[bestIndex] = tempRouteList[bestIndex].current_route;
			if(deltedRoute.Is_Empty())
			{//���ɾ��POI��·��Ϊ�գ�ɾ����·��
				routelist.erase(iteratorDeletedRoute);
			}
		}
		delete[] tempRouteList;
		tempRouteList = NULL;
	}//����������

	//���½�ĳ�Ա����
	this->UpdateStatus();
	this->List_to_RouteMatrix();
}

void CSolution::Random_Delete_Multiple_POI(int m)
{
	if(m < 1 || m > topo.itotalPOINumber - 1)
	{
		cout<<"Invalid POI Count";
		system("pause");
	}
	int *poiList = new int[topo.itotalPOINumber - 1];
	for(int i = 0;i < topo.itotalPOINumber - 1;i++)
	{
		poiList[i] = i + 1;
	}
	std::random_shuffle(poiList,poiList + topo.itotalPOINumber - 1);
	std::sort(poiList,poiList + m);

	LISTROUTE backup = routelist;//����routelist
	for(int i = 0;i < m;i++)
	{//ɾ��m��POI
		for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
		{
			bool found = false;
			for(LISTINT::iterator itPOI = it->seq.begin();itPOI != it->seq.end();itPOI++)
			{
				if(*itPOI == poiList[i])
				{
					CRoute temp(*it);
					if(!temp.DeleteFromRoute(*itPOI))
					{//ɾ��POIʧ��
						cout<<"DELETE POI FAILED IN DELETE M!\n";
						routelist = backup;//�ָ�routelist
						system("pause");
					}
					*it = temp.current_route;//����·��
					found = true;
					break;
				}
			}
			if(found)
			{
				break;
			}
		}
	}//ɾ�����

	//�������
	//std::random_shuffle(poiList,poiList + m);
	//for(int i = 0;i < m;i++)
	//{//����m��POI
	//	bool inserted = false;
	//	for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
	//	{
	//		CRoute temp(*it);
	//		bool success = temp.SolomenInsert(poiList[i]);
	//		if(!success)
	//		{
	//			continue;
	//		}
	//		else
	//		{//����ɹ�
	//			*it = temp.current_route;//����·��
	//			inserted = true;
	//			break;
	//		}
	//	}
	//	if(!inserted)
	//	{//����ʧ��
	//		cout<<"INSERT POI FAILED IN DELETE M!\n";
	//		routelist = backup;//�ָ�routelist
	//		system("pause");
	//	}
	//}//�������

	//ѡ�����������С��·������
	std::vector<LISTROUTE::iterator> pointers;
	for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
	{
		pointers.push_back(it);
	}
	std::random_shuffle(poiList,poiList + m);
	CRoute *tempRouteList = new CRoute[pointers.size()];//��ѡ����·���б�
	for(int i_poi = 0;i_poi < m;i_poi++)
	{
		int delta_sensor = 0;
		double delta_len = 0.0;
		int bestIndex = -1;//Ҫ����·�����±�
		for(unsigned int i = 0;i < pointers.size();i++)
		{
			tempRouteList[i].current_route = *pointers[i];
			bool inserted = tempRouteList[i].SolomenInsert(poiList[i_poi]);
			bool better = false;//���뵱ǰ·���Ƿ����
			if(inserted)
			{//���Բ���
				int temp_delta_sensor = tempRouteList[i].current_route.sensor_num - pointers[i]->sensor_num;
				double temp_delta_len = tempRouteList[i].current_route.len - pointers[i]->len;
				if(bestIndex == -1)
				{//����ǿ��Բ���ĵ�һ��·��
					better = true;
				}
				else
				{
					if(temp_delta_sensor < delta_sensor)
					{//���sensor����С
						better = true;
					}
					if((temp_delta_sensor == delta_sensor) && (temp_delta_len < delta_len))
					{//���senrsor���䣬����·������С
						better =true;
					}
				}

				if(better)
				{//�������
					bestIndex = i;
					delta_sensor = temp_delta_sensor;
					delta_len = temp_delta_len;
				}
			}
		}

		//�ҵ���õĲ���·��
		*pointers[bestIndex] = tempRouteList[bestIndex].current_route;
	}
	delete[] tempRouteList;
	tempRouteList = NULL;

	for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();)
	{//ɾ���ս�
		CRoute temp(*it);
		if(temp.Is_Empty())
		{
			it = routelist.erase(it);
		}
		else
		{
			it++;
		}
	}

	//���½�ĳ�Ա����
	this->UpdateStatus();
	this->List_to_RouteMatrix();

	delete[] poiList;
	poiList = NULL;
}

void CSolution::Random_Delete_POI_By_CoverageRate()
{
	LISTROUTE backup = routelist;//����routelist

	std::list<LISTROUTE::iterator> pointers;
	int sensors=this->sensor_num;
	printf("sensor_num is %d\n",sensors);
	double coverageRate = (double)(topo.itotalPOINumber - 1)/ (double)this->sensor_num ;
	printf("coverageRate is %f\n",coverageRate);
	for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
	{
		if(coverageRate > ((double)it->poi_num / it->sensor_num))
		{
			pointers.push_back(it);
		}
	}

	if(pointers.empty())
	{//���û��·������Ч�ʵ����ܸ���Ч��
		Random_Delete_One_POI();
	}
	else
	{//�Ӹ���Ч�ʵ͵�·�������ɾ��POI��������һ·��
		int routeNoInList = rand() % pointers.size();
		std::list<LISTROUTE::iterator>::iterator iteratorDeletedRoute = pointers.begin();//��������ָ��Ҫɾ��POI��·���ĵ�����
		for(int i = 0;i < routeNoInList;i++)
		{
			iteratorDeletedRoute++;
		}
		CRoute deltedRoute(**iteratorDeletedRoute);//��Ҫɾ��һ��POI��·���ĸ���
		BASIC_ROUTE_Struct routeForDel = **iteratorDeletedRoute;

		/*Randomly Slecte One POI for deleting*/
		int poiNoInSeq = rand() % (deltedRoute.current_route.seq.size() - 2);
		int selectedPOI = 0;
		LISTINT::iterator itPOI = deltedRoute.current_route.seq.begin();
		for(int i = -1;i < poiNoInSeq;i++)
		{
			itPOI++;
		}
		selectedPOI = *itPOI;
		if(selectedPOI <= 0) 
		{//ѡ�в��Ϸ�POI
			cout<<"POI INVALID!\n";
			system("pause");
		}
		bool deleted = deltedRoute.DeleteFromRoute(selectedPOI);

		/*Randomly Slecte One Route to insert*/
		LISTROUTE::iterator iteratorInsertedRoute = routelist.begin();
		if(!deleted)
		{//ɾ��POIʧ��
			cout<<"DELETE POI FAILED!\n";
			system("pause");
		}
		else
		{//ɾ���ɹ�
			**iteratorDeletedRoute = deltedRoute.current_route;//����ɾ��POI��·��
			std::vector<LISTROUTE::iterator> pointers;
			for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
			{
				pointers.push_back(it);
			}
			//std::random_shuffle(pointers.begin(),pointers.end());
			//bool sucInsert = false;
			//while(!pointers.empty())
			//{
			//	iteratorInsertedRoute = pointers.back();
			//	CRoute insertedRoute(*iteratorInsertedRoute);//��Ҫ����һ��POI��·���ĸ���
			//	bool inserted = insertedRoute.SolomenInsert(selectedPOI);
			//	if(!inserted)
			//	{//������ѡ���·�����ܲ���
			//		pointers.pop_back();
			//	}
			//	else
			//	{//����ɹ�������routelist
			//		if(deltedRoute.Is_Empty())
			//		{//���ɾ��POI��·��Ϊ�գ�ɾ����·��
			//			routelist.erase(*iteratorDeletedRoute);
			//		}
			//		*iteratorInsertedRoute = insertedRoute.current_route;//���²���POI��·��
			//		sucInsert = true;
			//		break;
			//	}
			//}

			//if(!sucInsert)
			//{//�Ҳ����ܲ����·��
			//	**iteratorDeletedRoute = routeForDel;
			//}

			//ѡ����õ�λ�ò���
			bool sucInsert = false;
			int delta_sesnor = 0;
			double delta_len = 0.0;
			int bestIndex = -1;//Ҫ����·�����±�
			CRoute *tempRouteList = new CRoute[pointers.size()];//��ѡ����·���б�
			for(unsigned int i = 0;i < pointers.size();i++)
			{
				tempRouteList[i].current_route = *pointers[i];
				bool inserted = tempRouteList[i].SolomenInsert(selectedPOI);
				bool better = false;
				if(inserted)
				{
					sucInsert = true;
					int temp_delta_sensor = tempRouteList[i].current_route.sensor_num - pointers[i]->sensor_num;
					double temp_delta_len = tempRouteList[i].current_route.len - pointers[i]->len;
					if(bestIndex == -1)
					{
						better = true;
					}
					else
					{
						if(temp_delta_sensor < delta_sesnor)
						{//���sensor����С
							better = true;
						}
						if((temp_delta_sensor == delta_sesnor) && (temp_delta_len < delta_len))
						{//���senrsor���䣬����·������С
							better =true;
						}
					}

					if(better)
					{
						bestIndex = i;
						delta_sesnor = temp_delta_sensor;
						delta_len = temp_delta_len;
					}
				}
			}

			if(!sucInsert)
			{//�Ҳ����ܲ����·��
				**iteratorDeletedRoute = routeForDel;
			}
			else
			{//�ҵ���õĲ���·��
				*pointers[bestIndex] = tempRouteList[bestIndex].current_route;
				if(deltedRoute.Is_Empty())
				{//���ɾ��POI��·��Ϊ�գ�ɾ����·��
					routelist.erase(*iteratorDeletedRoute);
				}
			}
			delete[] tempRouteList;
			tempRouteList = NULL;
		}//����������

		//���½�ĳ�Ա����
		this->UpdateStatus();
		this->List_to_RouteMatrix();
	}//�������
}

void CSolution::One_Point_Crossover()
{
	CSolution soll;
	LISTROUTE::iterator i,j,ri,rj;//ѡ��Ҫ����������·�� ri,rj���ڸ���i��j
	LISTINT::iterator k,l,g,h,temp;//k��l�ֱ��Ӧi��j�ķָ�ָ�룬ָ��ָ�㣬g��h������·���ϲ�
	CRoute m,n;//���γɵ�������·��
	list<Asetofpaths> la;
	list<Asetofpaths>::iterator las;
	Asetofpaths as,as_best;
	int flag=0;//���ڿ���·���ĵ���

	for(i=routelist.begin();i!=--routelist.end();/*i++*/)
	{
		if(flag==0)//��ʼ����ʱ��
		{
			j=i;
			j++;
		}
		else
		{
			if(flag==2)//��һ��δ���ָ��õ�·��,�ڶ���·����δ�������һ��·��,�ڶ���·�����(��һ)
			{
				j++;
			}
			else
			{
				if(flag==1)//�ڶ���·���ĵ������ﾡͷ�����Ե�һ��·��Ӧ�÷����ı�
				{
					i++;
					j=i;
					j++;
				}
				else
				{
					if(flag==4)//��һ�ַ����˸��õ�һ��·��,����֮��,������ʼ,i,j���ֲ���
					{}
				}
			}
		}
		if(i==--routelist.end())
			continue;
		for(k=++(*i).seq.begin();k!=----(*i).seq.end();k++)
		{
			for(l=++(*j).seq.begin();l!=----(*j).seq.end();l++)
			{
				m=CRoute();
				n=CRoute();
				temp=k;
				temp++;
				for(g=(*i).seq.begin();g!=temp;g++)
					m.current_route.seq.push_back(*g);
				temp=l;
				temp++;
				for(h=temp;h!=(*j).seq.end();h++)
					m.current_route.seq.push_back(*h);
				m.current_route.capacity=BUFF_MAX;
				m.UpdateStatus();
				if(m.current_route.capacity>=0)//����һ���Ϸ�·����˵�����Լ���
				{
					temp=l;
					temp++;
					for(h=(*j).seq.begin();h!=temp;h++)
						n.current_route.seq.push_back(*h);
					temp=k;
					temp++;
					for(g=temp;g!=(*i).seq.end();g++)
						n.current_route.seq.push_back(*g);
					n.current_route.capacity=BUFF_MAX;
					n.UpdateStatus();
					if(n.current_route.capacity>=0)//���ɵڶ����Ϸ�·�������������ƶ��ڵ���٣������ܳ��ȱ��٣����¼��list�ṹ��la����
					{
						/*n.UpdateStatus();*/
						//��������·������Ҫ���ƶ��ڵ���Ŀ
						//m.sensor_num=recomSen_num(m);
						//n.sensor_num=recomSen_num(n);
						as.mm=m.current_route;
						as.nn=n.current_route;
						as.sensor_num=m.current_route.sensor_num+n.current_route.sensor_num;
						if(((*i).sensor_num+(*j).sensor_num)>=(as.sensor_num))
						{
							as.all_len=m.current_route.len+n.current_route.len;
							if(((*i).sensor_num+(*j).sensor_num)>(as.sensor_num))
								la.push_back(as);
							else
							{
								if(((*i).len+(*j).len)>=as.all_len)
									la.push_back(as);
							}

						}
					}
					else
					{
						continue;
					}
				}
				else
				{
					continue;
				}

			}
		}
		//�����ڱȵ�ǰ����·���õ�ѡ����ѡ������õģ�����������
		if(!la.empty())
		{
			as_best=*la.begin();
			for(las=la.begin();las!=la.end();las++)
			{
				if((*las).sensor_num<as_best.sensor_num)
				{
					as_best=*las;
				}
				else
				{
					if((*las).sensor_num==as_best.sensor_num)
					{
						if((*las).all_len<as_best.all_len)
						{
							as_best=*las;
						}
					}
				}
			}
			la.clear();
			ri=i;
			ri++;
			routelist.insert(ri,as_best.mm);
			i=routelist.erase(i);
			rj=j;
			rj++;
			routelist.insert(rj,as_best.nn);
			j=routelist.erase(j);
			flag=4;
		}
		else
		{
			if(++j==routelist.end())
			{
				flag=1;
			}
			else
			{
				flag=2;
			}
			j--;
		}
	}
	UpdateStatus();
}

void CSolution::Two_Point_Crossover()
{
	CSolution soll;
	LISTROUTE::iterator i,j,ri,rj;//ѡ��Ҫ����������·�� ri,rj���ڸ���i��j
	LISTINT::iterator k,l,g,h,temp;//k��l�ֱ��Ӧi��j�ķָ�ָ�룬ָ��ָ�㣬g��h������·���ϲ�
	CRoute m,n;//���γɵ�������·��
	list<Asetofpaths> la;
	list<Asetofpaths>::iterator las;
	Asetofpaths as,as_best;
	int flag=0;//���ڿ���·���ĵ���
	int bag[4],count=0;
	for(i=routelist.begin();i!=--routelist.end();/*i++*/)
	{

		if(flag==0)
		{
			j=i;
			j++;
		}
		else
		{
			if(flag==2)
			{
				j++;
			}
			else
			{
				if(flag==1)
				{
					i++;
					j=i;
					j++;
				}
				else
				{
					if(flag==4)
					{}
				}
			}
		}
		if(i==--routelist.end())
			continue;
		if((*i).seq.size()<=3)
		{
			flag=1;
			continue;
		}
		else
		{
			if((*j).seq.size()<=3)
			{
				if(++j==routelist.end())
				{
					flag=1;
				}
				else
				{
					flag=2;
				}
				j--;
				continue;
			}
		}
		for(k=++(*i).seq.begin();k!=----(*i).seq.end();k++)
		{

			for(l=++(*j).seq.begin();l!=----(*j).seq.end();l++)
			{
				bag[0]=*k;
				bag[1]=*l;
				temp=l;
				temp++;
				bag[2]=*temp;
				temp=k;
				temp++;
				bag[3]=*temp;
				for(count=0;count<5;count++)
				{
					m=CRoute();
					n=CRoute();
					for(g=(*i).seq.begin();g!=k;g++)
						m.current_route.seq.push_back(*g);
					if(count==0)
					{
					}
					if(count==1)
					{
						m.current_route.seq.push_back(bag[0]);
					}
					if(count==2)
					{
						m.current_route.seq.push_back(bag[0]);
						m.current_route.seq.push_back(bag[1]);
					}
					if(count==3)
					{
						m.current_route.seq.push_back(bag[0]);
						m.current_route.seq.push_back(bag[1]);
						m.current_route.seq.push_back(bag[2]);
					}
					if(count==4)
					{
						m.current_route.seq.push_back(bag[0]);
						m.current_route.seq.push_back(bag[1]);
						m.current_route.seq.push_back(bag[2]);
						m.current_route.seq.push_back(bag[3]);
					}
					temp=k;
					temp++;
					temp++;
					for(g=temp;g!=(*i).seq.end();g++)
					{
						m.current_route.seq.push_back(*g);
					}
					m.current_route.capacity=BUFF_MAX;
					m.UpdateStatus();
					if(m.current_route.capacity>=0)
					{
						for(h=(*j).seq.begin();h!=l;h++)
							n.current_route.seq.push_back(*h);
						if(count==0)
						{
							n.current_route.seq.push_back(bag[0]);
							n.current_route.seq.push_back(bag[1]);
							n.current_route.seq.push_back(bag[2]);
							n.current_route.seq.push_back(bag[3]);
						}
						if(count==1)
						{
							n.current_route.seq.push_back(bag[1]);
							n.current_route.seq.push_back(bag[2]);
							n.current_route.seq.push_back(bag[3]);
						}
						if(count==2)
						{
							n.current_route.seq.push_back(bag[2]);
							n.current_route.seq.push_back(bag[3]);
						}
						if(count==3)
						{
							n.current_route.seq.push_back(bag[3]);
						}
						if(count==4)
						{
						}
						temp=l;
						temp++;
						temp++;
						for(h=temp;h!=(*j).seq.end();h++)
						{
							n.current_route.seq.push_back(*h);
						}
						n.current_route.capacity=BUFF_MAX;
						n.UpdateStatus();
						if(n.current_route.capacity>=0)
						{
							as.mm=m.current_route;
							as.nn=n.current_route;
							as.sensor_num=m.current_route.sensor_num+n.current_route.sensor_num;
							if(((*i).sensor_num+(*j).sensor_num)>=(as.sensor_num))
							{
								as.all_len=m.current_route.len+n.current_route.len;
								if(((*i).sensor_num+(*j).sensor_num)>(as.sensor_num))
									la.push_back(as);
								else
								{
									if(((*i).len+(*j).len)>=as.all_len)
										la.push_back(as);
								}
							}
						}
						else
						{
							continue;
						}
					}
					else
					{
						continue;
					}

				}

			}
		}
		if(!la.empty())
		{
			as_best=*la.begin();
			for(las=la.begin();las!=la.end();las++)
			{
				if((*las).sensor_num<as_best.sensor_num)
					as_best=*las;
				else
				{
					if((*las).sensor_num==as_best.sensor_num)
					{
						if((*las).all_len<as_best.all_len)
							as_best=*las;
					}
				}
			}
			la.clear();
			ri=i;
			ri++;
			routelist.insert(ri,as_best.mm);
			i=routelist.erase(i);
			rj=j;
			rj++;
			routelist.insert(rj,as_best.nn);
			j=routelist.erase(j);
			flag=4;
		}
		else
		{
			if(++j==routelist.end())
			{
				flag=1;
			}
			else
			{
				flag=2;
			}
			j--;
		}
	}
	for(i=routelist.begin();i!=routelist.end();i++)
	{
		if((*i).seq.size()>2)
			soll.routelist.push_back((*i));
	}
	routelist=soll.routelist;
	UpdateStatus();
}

void CSolution::Additional_Local_Search()
{
	CSolution tempsol;
	LISTINT templist;
	LISTROUTE::iterator i,j,k,l;
	LISTINT::iterator m,n,x;
	int fewest=topo.itotalPOINumber,s_num=10,isend=0;
	int size=0;
	bool isContinue=true;
	tempsol.routelist=routelist;
	tempsol.UpdateStatus();
	while(isContinue)
	{
		fewest=topo.itotalPOINumber;
		s_num=10;size=0;
		j=tempsol.routelist.begin();

		for(i=tempsol.routelist.begin();i!=tempsol.routelist.end();i++)
		{
			if((*i).sensor_num<s_num)
			{
				s_num=(*i).sensor_num;
				if((*i).poi_num<fewest)
				{
					fewest=(*i).poi_num;
					j=i;
				}
			}
		}


		templist=(*j).seq;
		tempsol.routelist.erase(j);

		for(m=++templist.begin();m!=--templist.end();m++)
		{
			isend=0;
			for(k=tempsol.routelist.begin();k!=tempsol.routelist.end();)
			{
				CRoute rt=CRoute(*k);
				l=k;
				for(n=(*k).seq.begin();n!=--(*k).seq.end();n++)
				{
					if(rt.IsFeasibleTimeAndBuffer(*m))
					{
						if(rt.SolomenInsert(*m))
						{
							size++;
							isend=1;
							break;
						}
					}
				}
				l++;
				tempsol.routelist.insert(l,rt.current_route);
				tempsol.routelist.erase(k);
				k=l;
				if(isend==1)
				{
					break;
				}
			}
		}

		if(size==(templist.size()-2))
		{
			routelist=tempsol.routelist;
			isContinue=true;
		}
		else
		{
			isContinue=false;
		}
	}
	//PrintSolution(Position[Particle_No]);

	UpdateStatus();
}

unsigned int  CSolution::RandomDeleteOnePOI(CSolution& uncompletedsolution)
{
	int deleteRouteSeq=RandomInt(0,uncompletedsolution.routelist.size());
	LISTROUTE::iterator oproute=uncompletedsolution.routelist.begin();
	for(int i=0;i<=deleteRouteSeq;i++)
		oproute++;
	CRoute operating_route=CRoute((*oproute));
	int deletePOI_Seq=RandomInt(1,(*oproute).seq.size()-1);
	int deletePOI_ID=0;
	LISTINT::iterator poipointer=(*oproute).seq.begin();
	for (int i=0;i<=deletePOI_Seq;i++)
		poipointer++;
	deletePOI_ID=(*poipointer);
	operating_route.DeleteFromRoute(deletePOI_ID);
	operating_route.UpdateStatus();
	(*oproute)=operating_route.current_route;

	if((*oproute).seq.size()<=2)
	{
		uncompletedsolution.routelist.erase(oproute);
	}
	uncompletedsolution.UpdateStatus();
	return (unsigned)deletePOI_ID;
}

unsigned int  CSolution::RandomDeleteOnePOI_ContactRate(CSolution& uncompletedsolution)
{
	list<LISTROUTE::iterator> candidate_route;
	int poinumber=uncompletedsolution.GetPOINumber();
	double coverageRate = (double)(poinumber)/ (double)uncompletedsolution.sensor_num ;
	//buiding candidate route set
	for(LISTROUTE::iterator it = uncompletedsolution.routelist.begin();it !=uncompletedsolution.routelist.end();it++)
	{
		if(coverageRate > ((double)it->poi_num / it->sensor_num))
		{
			candidate_route.push_back(it);
		}
	}
	if(candidate_route.empty())
	{
		return RandomDeleteOnePOI(uncompletedsolution);
	}
	int deleteRouteSeq=RandomInt(0,candidate_route.size());
	list<LISTROUTE::iterator>::iterator oproute=candidate_route.begin();
	for(int i=0;i<deleteRouteSeq;i++)
		oproute++;
	CRoute operating_route=CRoute((**oproute));
	//cout<<"seq.size() is "<<(**oproute).seq.size()<<endl;
	int deletePOI_Seq=RandomInt(1,(**oproute).seq.size()-1);
	int deletePOI_ID=0;
	LISTINT::iterator poipointer=(**oproute).seq.begin();
	for (int i=0;i<=deletePOI_Seq;i++)
		poipointer++;
	deletePOI_ID=(*poipointer);
	operating_route.DeleteFromRoute(deletePOI_ID);
	operating_route.UpdateStatus();
	(**oproute)=operating_route.current_route;
	if(operating_route.Is_Empty())
	{
		uncompletedsolution.routelist.erase(*oproute);
	}
	uncompletedsolution.UpdateStatus();
	return (unsigned)deletePOI_ID;
}
int CSolution::GetPOINumber()
{
	int result=0;
	for(LISTROUTE::iterator it = this->routelist.begin();it !=this->routelist.end();it++)
	{
		result+=(*it).poi_num;
	}
	return result;
}
bool CSolution::Random_Insert_One_POI(CSolution& uncompletedsolution,unsigned int poiid)
{
	const int times=10;
	int i=0;
	while(i<times)
	{
		int opRouteSeq=RandomInt(0,uncompletedsolution.routelist.size());
		LISTROUTE::iterator oproute=uncompletedsolution.routelist.begin();
		for(int i=0;i<opRouteSeq;i++)
			oproute++;
		CRoute operating_route=CRoute((*oproute));
		if(operating_route.IsFeasible(poiid))
		{
			operating_route.SolomenInsert(poiid);
			operating_route.UpdateStatus();
			(*oproute)=operating_route.current_route;
			cout<<"after insert"<<endl;
			uncompletedsolution.UpdateStatus();
			return true;
		}
	}
	//�Ҳ������ʵ�route������õ�ѡ��
	return Insert_Best_Route(uncompletedsolution,poiid);
}
bool CSolution::Insert_Best_Route(CSolution& uncompletedsolution,unsigned int poiid)
{
	CSolution s=CSolution(uncompletedsolution);
	list<CRoute> CRoute_list;
	list<CRoute>::iterator op_route;
	LISTROUTE::iterator pointer;
	for(pointer=s.routelist.begin();pointer!=s.routelist.end();pointer++)
	{
		CRoute operating_route=CRoute(*pointer);
		CRoute_list.push_back(operating_route);
	}
	bool success=false;
	bool better=false;
	double leastlength=0;
	double leastlen=0;
	int i=0;
	int min_target=0;
	pointer=s.routelist.begin();
	for(op_route=CRoute_list.begin();op_route!=CRoute_list.end();op_route++)
	{

		if ((*op_route).IsFeasible(poiid))
		{

			(*op_route).SolomenInsert(poiid);
			(*op_route).UpdateStatus();
			double temp=(*op_route).current_route.len-(*pointer).len;
			success=true;
			if((*op_route).current_route.sensor_num==(*pointer).sensor_num)
			{
				better=true;
			
				temp=temp/((*op_route).current_route.sensor_num);
				//the first better route
				if(leastlength==0)
				{
					leastlength=temp;
					min_target=i;

				}
				else
				{
					if(leastlength>temp)
					{
						leastlength=temp;
						min_target=i;
					}
				}//end of leastlength
				//DBG:
				//printf("Better\n");
			}//end of finding routes without adding sensors
			else
			{
				//Failed to find a route without add sensor yet!
				if (!better)
				{
					if(leastlen==0)
					{
						leastlen=temp;
						min_target=i;
					}
					else
					{
						if(leastlen>temp)
						{
							leastlen=temp;
							min_target=i;
						}
					}//end of leastlen
				}// end of better
				//DBG:
				//printf("Not better.\n");

			}//end of finding routes adding sensors
		}//end of isfeasible()
		pointer++;
		i++;
		//DBG:
		//printf("Successful.\n");

	}// end of for
	//failed to find a route to insert, create a new route
	if (!success)
	{
		BASIC_ROUTE_Struct newroute;
		CRoute crt=CRoute();
		crt.current_route.seq.push_back(0);
		crt.current_route.seq.push_back(0);
		crt.InsertAfter(0,poiid);
		crt.UpdateStatus();
		newroute=crt.current_route;
		uncompletedsolution.routelist.push_back(newroute);
		uncompletedsolution.UpdateStatus();
		uncompletedsolution.List_to_RouteMatrix();
		//DBG:
		//printf("Not successful. Create a new route.\n");

		return true;	
	}
	//DBG:
	//printf("**Insert poi %u to route %d.\n", poiid, min_target);

	//Insert to uncompletedsolution
	pointer=uncompletedsolution.routelist.begin();
	for(int j=0;j<min_target;j++)
		pointer++;
	CRoute selected_route=CRoute(*pointer);
	selected_route.SolomenInsert(poiid);
	selected_route.UpdateStatus();
	(*pointer)=selected_route.current_route;
	uncompletedsolution.UpdateStatus();
	return true;
}