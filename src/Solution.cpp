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
	//cout<<"CSolution 拷贝构造"<<endl;
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
	//cout<<"operator= 号 重载 "<<endl;
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
	//cout<<"CSolution pRouteMatrix 析构"<<endl;
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
	double temp;//行的最大概率
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
		for(j=0;j<topo.itotalPOINumber;j++)//归一化
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
		for(j=list_nid.begin();j!=list_nid.end();++j)		//顺序输出节点。与访问先后一一对应。
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
	CRoute deltedRoute(*iteratorDeletedRoute);//将要删除一个POI的路径的副本
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
	{//选中不合法POI
		cout<<"POI INVALID!\n";
		system("pause");
	}
	bool deleted = deltedRoute.DeleteFromRoute(selectedPOI);

	/*Randomly Slecte One Route to insert*/
	LISTROUTE::iterator iteratorInsertedRoute = routelist.begin();
	if(!deleted)
	{//删除POI失败
		cout<<"DELETE POI FAILED!\n";
		system("pause");
	}
	else
	{//删除成功
		*iteratorDeletedRoute = deltedRoute.current_route;//更新删除POI的路径
		std::vector<LISTROUTE::iterator> pointers;
		for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
		{
			pointers.push_back(it);
		}
		//随机插入
		//bool sucInsert = false;
		//std::random_shuffle(pointers.begin(),pointers.end());
		//while(!pointers.empty())
		//{
		//	iteratorInsertedRoute = pointers.back();
		//	CRoute insertedRoute(*iteratorInsertedRoute);//将要插入一个POI的路径的副本
		//	bool inserted = insertedRoute.SolomenInsert(selectedPOI);
		//	if(!inserted)
		//	{//如果随机选择的路径不能插入
		//		pointers.pop_back();
		//	}
		//	else
		//	{//插入成功，更新routelist
		//		if(deltedRoute.Is_Empty())
		//		{//如果删除POI后路径为空，删除该路径
		//			routelist.erase(iteratorDeletedRoute);
		//		}
		//		*iteratorInsertedRoute = insertedRoute.current_route;//更新插入POI的路径
		//		sucInsert = true;
		//		break;
		//	}
		//}
		//if(!sucInsert)
		//{//找不到能插入的路径
		//	*iteratorDeletedRoute = routeForDel;
		//}

		//选择最好的位置插入
		bool sucInsert = false;
		int delta_sesor = 0;
		double delta_len = 0.0;
		int bestIndex = -1;//要插入路径的下标
		CRoute *tempRouteList = new CRoute[pointers.size()];//候选插入路径列表
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
					{//如果sensor增量小
						better = true;
					}
					if((temp_delta_sesor == delta_sesor) && (temp_delta_len < delta_len))
					{//如果senrsor不变，但是路径增量小
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
		{//找不到能插入的路径
			*iteratorDeletedRoute = routeForDel;
		}
		else
		{//找到最好的插入路径
			*pointers[bestIndex] = tempRouteList[bestIndex].current_route;
			if(deltedRoute.Is_Empty())
			{//如果删除POI后路径为空，删除该路径
				routelist.erase(iteratorDeletedRoute);
			}
		}
		delete[] tempRouteList;
		tempRouteList = NULL;
	}//插入操作完成

	//更新解的成员属性
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

	LISTROUTE backup = routelist;//备份routelist
	for(int i = 0;i < m;i++)
	{//删除m个POI
		for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
		{
			bool found = false;
			for(LISTINT::iterator itPOI = it->seq.begin();itPOI != it->seq.end();itPOI++)
			{
				if(*itPOI == poiList[i])
				{
					CRoute temp(*it);
					if(!temp.DeleteFromRoute(*itPOI))
					{//删除POI失败
						cout<<"DELETE POI FAILED IN DELETE M!\n";
						routelist = backup;//恢复routelist
						system("pause");
					}
					*it = temp.current_route;//更新路径
					found = true;
					break;
				}
			}
			if(found)
			{
				break;
			}
		}
	}//删除完成

	//随机插入
	//std::random_shuffle(poiList,poiList + m);
	//for(int i = 0;i < m;i++)
	//{//插入m个POI
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
	//		{//插入成功
	//			*it = temp.current_route;//更新路径
	//			inserted = true;
	//			break;
	//		}
	//	}
	//	if(!inserted)
	//	{//插如失败
	//		cout<<"INSERT POI FAILED IN DELETE M!\n";
	//		routelist = backup;//恢复routelist
	//		system("pause");
	//	}
	//}//插入完成

	//选择插入增量最小的路径插入
	std::vector<LISTROUTE::iterator> pointers;
	for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();it++)
	{
		pointers.push_back(it);
	}
	std::random_shuffle(poiList,poiList + m);
	CRoute *tempRouteList = new CRoute[pointers.size()];//候选插入路径列表
	for(int i_poi = 0;i_poi < m;i_poi++)
	{
		int delta_sensor = 0;
		double delta_len = 0.0;
		int bestIndex = -1;//要插入路径的下标
		for(unsigned int i = 0;i < pointers.size();i++)
		{
			tempRouteList[i].current_route = *pointers[i];
			bool inserted = tempRouteList[i].SolomenInsert(poiList[i_poi]);
			bool better = false;//插入当前路径是否更优
			if(inserted)
			{//可以插入
				int temp_delta_sensor = tempRouteList[i].current_route.sensor_num - pointers[i]->sensor_num;
				double temp_delta_len = tempRouteList[i].current_route.len - pointers[i]->len;
				if(bestIndex == -1)
				{//如果是可以插入的第一条路径
					better = true;
				}
				else
				{
					if(temp_delta_sensor < delta_sensor)
					{//如果sensor增量小
						better = true;
					}
					if((temp_delta_sensor == delta_sensor) && (temp_delta_len < delta_len))
					{//如果senrsor不变，但是路径增量小
						better =true;
					}
				}

				if(better)
				{//如果更优
					bestIndex = i;
					delta_sensor = temp_delta_sensor;
					delta_len = temp_delta_len;
				}
			}
		}

		//找到最好的插入路径
		*pointers[bestIndex] = tempRouteList[bestIndex].current_route;
	}
	delete[] tempRouteList;
	tempRouteList = NULL;

	for(LISTROUTE::iterator it = routelist.begin();it != routelist.end();)
	{//删除空解
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

	//更新解的成员属性
	this->UpdateStatus();
	this->List_to_RouteMatrix();

	delete[] poiList;
	poiList = NULL;
}

void CSolution::Random_Delete_POI_By_CoverageRate()
{
	LISTROUTE backup = routelist;//备份routelist

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
	{//如果没有路径覆盖效率低于总覆盖效率
		Random_Delete_One_POI();
	}
	else
	{//从覆盖效率低的路径中随机删除POI并插入任一路径
		int routeNoInList = rand() % pointers.size();
		std::list<LISTROUTE::iterator>::iterator iteratorDeletedRoute = pointers.begin();//迭代器，指向要删除POI的路径的迭代器
		for(int i = 0;i < routeNoInList;i++)
		{
			iteratorDeletedRoute++;
		}
		CRoute deltedRoute(**iteratorDeletedRoute);//将要删除一个POI的路径的副本
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
		{//选中不合法POI
			cout<<"POI INVALID!\n";
			system("pause");
		}
		bool deleted = deltedRoute.DeleteFromRoute(selectedPOI);

		/*Randomly Slecte One Route to insert*/
		LISTROUTE::iterator iteratorInsertedRoute = routelist.begin();
		if(!deleted)
		{//删除POI失败
			cout<<"DELETE POI FAILED!\n";
			system("pause");
		}
		else
		{//删除成功
			**iteratorDeletedRoute = deltedRoute.current_route;//更新删除POI的路径
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
			//	CRoute insertedRoute(*iteratorInsertedRoute);//将要插入一个POI的路径的副本
			//	bool inserted = insertedRoute.SolomenInsert(selectedPOI);
			//	if(!inserted)
			//	{//如果随机选择的路径不能插入
			//		pointers.pop_back();
			//	}
			//	else
			//	{//插入成功，更新routelist
			//		if(deltedRoute.Is_Empty())
			//		{//如果删除POI后路径为空，删除该路径
			//			routelist.erase(*iteratorDeletedRoute);
			//		}
			//		*iteratorInsertedRoute = insertedRoute.current_route;//更新插入POI的路径
			//		sucInsert = true;
			//		break;
			//	}
			//}

			//if(!sucInsert)
			//{//找不到能插入的路径
			//	**iteratorDeletedRoute = routeForDel;
			//}

			//选择最好的位置插入
			bool sucInsert = false;
			int delta_sesnor = 0;
			double delta_len = 0.0;
			int bestIndex = -1;//要插入路径的下标
			CRoute *tempRouteList = new CRoute[pointers.size()];//候选插入路径列表
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
						{//如果sensor增量小
							better = true;
						}
						if((temp_delta_sensor == delta_sesnor) && (temp_delta_len < delta_len))
						{//如果senrsor不变，但是路径增量小
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
			{//找不到能插入的路径
				**iteratorDeletedRoute = routeForDel;
			}
			else
			{//找到最好的插入路径
				*pointers[bestIndex] = tempRouteList[bestIndex].current_route;
				if(deltedRoute.Is_Empty())
				{//如果删除POI后路径为空，删除该路径
					routelist.erase(*iteratorDeletedRoute);
				}
			}
			delete[] tempRouteList;
			tempRouteList = NULL;
		}//插入操作完成

		//更新解的成员属性
		this->UpdateStatus();
		this->List_to_RouteMatrix();
	}//插入完成
}

void CSolution::One_Point_Crossover()
{
	CSolution soll;
	LISTROUTE::iterator i,j,ri,rj;//选择要操作的两条路径 ri,rj用于更新i，j
	LISTINT::iterator k,l,g,h,temp;//k，l分别对应i，j的分割指针，指向分割点，g，h用于新路径合并
	CRoute m,n;//新形成的两条新路径
	list<Asetofpaths> la;
	list<Asetofpaths>::iterator las;
	Asetofpaths as,as_best;
	int flag=0;//用于控制路径的迭代

	for(i=routelist.begin();i!=--routelist.end();/*i++*/)
	{
		if(flag==0)//初始化的时候
		{
			j=i;
			j++;
		}
		else
		{
			if(flag==2)//上一轮未发现更好的路径,第二条路径还未到达最后一条路径,第二条路径变更(加一)
			{
				j++;
			}
			else
			{
				if(flag==1)//第二条路径的迭代到达尽头，所以第一条路经应该发生改变
				{
					i++;
					j=i;
					j++;
				}
				else
				{
					if(flag==4)//上一轮发现了更好的一对路径,更新之后,继续开始,i,j保持不变
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
				if(m.current_route.capacity>=0)//生成一条合法路径，说明可以继续
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
					if(n.current_route.capacity>=0)//生成第二条合法路径，若满足总移动节点变少，或者总长度变少，则记录在list结构体la里面
					{
						/*n.UpdateStatus();*/
						//计算两条路径所需要的移动节点数目
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
		//若存在比当前两条路径好的选择，则选其中最好的，更新两条边
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
	LISTROUTE::iterator i,j,ri,rj;//选择要操作的两条路径 ri,rj用于更新i，j
	LISTINT::iterator k,l,g,h,temp;//k，l分别对应i，j的分割指针，指向分割点，g，h用于新路径合并
	CRoute m,n;//新形成的两条新路径
	list<Asetofpaths> la;
	list<Asetofpaths>::iterator las;
	Asetofpaths as,as_best;
	int flag=0;//用于控制路径的迭代
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
	//找不到合适的route，找最好的选择
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