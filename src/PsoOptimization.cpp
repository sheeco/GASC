#include "PsoOptimization.h"

extern CTopology topo;

PsoOptimization::PsoOptimization(void)
{
	unsigned int i;
	NumberofSolution=Particle_Num;
	Weight=0;
	pSolution=new CSolution[NumberofSolution];
	for(i=0;i<NumberofSolution;i++)
	{
		pSolution[i]=CSolution();
	}
	pBestSolution=new CSolution[NumberofSolution];
	for(i=0;i<NumberofSolution;i++)
	{
		pBestSolution[i]=CSolution();
	}
	gBestSolution=CSolution();
	p=NULL;
	/*初始化p矩阵，用于存储每条边被选中的概率p（i，j）*/
	p=new double**[NumberofSolution];
	for(int k=0;k<NumberofSolution;k++)
	{
		p[k]=new double*[topo.itotalPOINumber];
		for(int x=0;x<topo.itotalPOINumber;x++)
		{
			p[k][x]=new double[topo.itotalPOINumber];
		}
	}

	for(i=0;i<NumberofSolution;i++)
	{
		initPbytl(i);
		//initP(i);
		//rdinitP(i);
	}
}


PsoOptimization::~PsoOptimization(void)
{
	//if (pDistanceMatrix!=NULL)
	//{
	//	for(int i=0;i<itotalPOINumber;i++)
	//	{
	//		delete[] pDistanceMatrix[i];
	//	}
	//}
	//delete[] pDistanceMatrix;
	//pDistanceMatrix=NULL;
	//for(int k=0;k<NumberofSolution;k++)
	//{
	//	p[k]=new double*[topo.itotalPOINumber];
	//	for(int x=0;x<topo.itotalPOINumber;x++)
	//	{
	//		p[k][x]=new double[topo.itotalPOINumber];
	//	}
	//}
	temPOI.clear();
	Weight=0;
	cout<<"p矩阵析构"<<endl;
	if(p!=NULL)
	{
		for(int i=0;i<NumberofSolution;i++)
		{
			for(int j=0;j<topo.itotalPOINumber;j++)
			{
				delete[]p[i][j];
			}
			delete[]p[i];
		}
	}
	delete[]p;
	p=NULL;
	if(pBestSolution!=NULL)
	{
		delete[]pBestSolution;
	}
	pBestSolution=NULL;
	
}

void PsoOptimization::GenerateInit(void)
{
	int i;

	for(i=0;i<NumberofSolution;i++)
	{
		pSolution[i]=Route_Building(i);
		//pSolution[i].PrintSolution();
		pSolution[i].LocalOptimize();
		pSolution[i].List_to_RouteMatrix();
		pSolution[i].UpdateStatus();
		pSolution[i].route_num=pSolution[i].routelist.size();
		pBestSolution[i]=pSolution[i];
		//pSolution[i].PrintSolution();


		//pSolution[i].CalculateSensorNumber();

		//cout<<pSolution[i].IsPracticiable()<<endl;

		//for(int k=0;k<topo.itotalPOINumber;k++)
		//{
		//	for(int j=0;j<topo.itotalPOINumber;j++)
		//	{
		//		cout<<pSolution[i].pRouteMatrix[k][j]<<" ";
		//	}
		//	cout<<endl;
		//}
		//pSolution[i].RouteMatrix_to_List();
		////pSolution[i].PrintSolution();
		//cout<<endl;
		//cout<<endl;
	}
	gBestSolution=pBestSolution[0];
	updateGbest();
	gBestSolution.PrintSolution();
}

CSolution PsoOptimization::Optimize(void)
{
	int m,n,isend=0,isEmpty=0;
	unsigned int i,j;
	LISTINT tempEdge;
	LISTINT::iterator q,q2;
	LISTROUTE::iterator pr,pr2;
	double Psum=0;
	double rd;//公式（13）中的rand
	double rd1;//公式（13）中用于选择Pbest的随机值，小于阀值，随机选择Pbest
	double h;
	clock_t start,finish;
	int	pBest_f1,pBest_f2,current_Pbest_Id;//公式（13）中的f(i)函数，确定使用第pBest_f个pBest
	int current_POI_id,front_POI_id,front_POI_id2,next_POI_id2,next_POI_id;
	start=clock();
	for(j=0;j<outer;j++)
	{
		Weight=0.4+0.5*(outer-j)/outer;
		//start=clock();
		for(i=0;i<NumberofSolution;i++)
		{
			rd=(double)rand() / RAND_MAX;
			rd1=(double)rand() / RAND_MAX;

			//for(m=0;m<Node_num;m++)
			//{
			//	for(n=0;n<Node_num;n++)
			//	{
			//		cout<<p[i][m][n]<<" ";
			//	}
			//	cout<<endl;
			//	cout<<endl;
			//}

			//velocity更新操作，这里就是对p矩阵进行操作，修改其p值
			for(m=0;m<topo.itotalPOINumber;m++)
			{
				for(n=0;n<topo.itotalPOINumber;n++)
				{
					if(n!=m)
					{
						p[i][m][n]=Weight*p[i][m][n]>1?1:Weight*p[i][m][n];
						//p[i][n][m]=Weight*p[i][n][m]>1?1:Weight*p[i][n][m];
					}
					else
						p[i][m][n]=0;
				}
			}

			isEmpty=0;
			for(pr=pSolution[i].routelist.begin();pr!=pSolution[i].routelist.end();pr++)
			{
				for(q=++(*pr).seq.begin();q!=--(*pr).seq.end();q++)
				{
					current_POI_id=(*q);
					//获得当前解中POI对应的id前后的POI的id
					q--;
					front_POI_id=*q;
					q++;q++;
					next_POI_id=*q;
					q--;
					if(i==2)
						h=1;
					if(rd1>Pc)
					{
						current_Pbest_Id=i;
					}
					else
					{
						pBest_f1=rand()%NumberofSolution;
						pBest_f2=rand()%NumberofSolution;
						if(pBestSolution[pBest_f1].sensor_num>pBestSolution[pBest_f2].sensor_num)
						{
							current_Pbest_Id=pBest_f2;
						}
						else
						{	
							if(pBestSolution[pBest_f1].sensor_num<pBestSolution[pBest_f2].sensor_num)
							{
								current_Pbest_Id=pBest_f1;
							}
							else
							{
								if(pBestSolution[pBest_f1].sensor_num==pBestSolution[pBest_f2].sensor_num)
								{
									if(reComputeAllLength(pBestSolution[pBest_f1])<=reComputeAllLength(pBestSolution[pBest_f2]))
										current_Pbest_Id=pBest_f1;
									else
									{
										current_Pbest_Id=pBest_f2;
									}
								}
								else
								{
									current_Pbest_Id=pBest_f2;
								}
							}
						}
					}
					//找到目标POI点标识的前一个和后一个节点的id
					isend=0;
					for(pr2=pBestSolution[current_Pbest_Id].routelist.begin();pr2!=pBestSolution[current_Pbest_Id].routelist.end();pr2++)
					{
						for(q2=++(*pr2).seq.begin();q2!=--(*pr2).seq.end();q2++)
						{
							if(current_POI_id==(*q2))
							{
								--q2;
								front_POI_id2=*(q2);
								q2++;q2++;
								next_POI_id2=(*q2);
								q2--;
								isend=1;
								break;
							}
						}
						if(isend>0)
							break;
					}
					h=Coefﬁcients*rd>1?1:Coefﬁcients*rd;
					if(front_POI_id2!=front_POI_id)
					{
						p[i][front_POI_id2][current_POI_id]=p[i][front_POI_id2][current_POI_id]>h?p[i][front_POI_id2][current_POI_id]:h;
						isEmpty++;
					}
					//if(next_POI_id2!=next_POI_id)
					//{
					//	p[i][current_POI_id][next_POI_id2]=p[i][current_POI_id][next_POI_id2]>h?p[i][current_POI_id][next_POI_id2]:h;
					//	isEmpty++;
					//}

				}
			}
			if(isEmpty==0)
			{
				//count_No_Change[i]++;
			}
			////P矩阵每行归一化
			for(m=0;m<topo.itotalPOINumber;m++)
			{
				Psum=0;
				for(n=0;n<topo.itotalPOINumber;n++)
					Psum+=p[i][m][n];
				for(n=0;n<topo.itotalPOINumber;n++)
					p[i][m][n]=p[i][m][n]/Psum;	
			}
			//再根据其概率大小，从p中选边。重新构成新路径，即粒子群中每个position的更新
			//cout<<"----------------------------"<<endl;
			//for(m=0;m<Node_num;m++)
			//{
			//	for(n=0;n<Node_num;n++)
			//	{
			//		cout<<p[i][m][n]<<" ";
			//	}
			//	cout<<endl;
			//}
			pSolution[i]=Route_Building(i);
			//pSolution[i].PrintSolution();
			pSolution[i].LocalOptimize();
			pSolution[i].List_to_RouteMatrix();
			//if(i==0)
			//cout<<"Pbest         :"<<i<<endl;
			//PrintSolution(Pbest[i]);
		}
		updatePbest();
		updateGbest();
	}
	finish=clock();
	cout<<"Optimize 的 执行时间为："<<(finish-start)<<endl;
	return gBestSolution;
}


void PsoOptimization::initP(int Particle_No)
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
				p[Particle_No][k][j]=0;
			else
			{
				p[Particle_No][k][j]=0;
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
				p[Particle_No][k][j]=0;
			else
			{
				p[Particle_No][k][j]=(double)(Max_len[k]-topo.GetDistance(k,j))*10/All_Len[k]/10;
				//cout<<p[Particle_No][k][j]<<endl;
			}
		}
	}
}


void PsoOptimization::initPbytl(int Particle_No)
{
	int k,j;
	double Psum,Tij;
	for(k=0;k<topo.itotalPOINumber;k++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			if(k==j)
				p[Particle_No][k][j]=0;
			else
			{
				//p[Particle_No][k][j]=(double)rand()/RAND_MAX;
				Tij=abs(topo.get_single_POI(k).d_time-topo.get_single_POI(j).d_time);
				Psum=Tij-topo.GetDistance(k,j)/M_VELOCITY;
				if(Psum>0)
					p[Particle_No][k][j]=(double)Psum/Tij;
				else
				{
					p[Particle_No][k][j]=0;
				}
			}
		}
	}
	for(k=0;k<topo.itotalPOINumber;k++)
	{
		Psum=0;
		for(j=0;j<topo.itotalPOINumber;j++)
			Psum+=p[Particle_No][k][j];
		for(j=0;j<topo.itotalPOINumber;j++)
			p[Particle_No][k][j]=p[Particle_No][k][j]/Psum;	
	}
}

void PsoOptimization::rdinitP(int Particle_No)
{
	int k,j;
	double Psum;
	for(k=0;k<topo.itotalPOINumber;k++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			if(k==j)
				p[Particle_No][k][j]=0;
			else
			{
				p[Particle_No][k][j]=(double)rand()/RAND_MAX;
			}
		}
	}
	//归一化
	for(k=0;k<topo.itotalPOINumber;k++)
	{
		Psum=0;
		for(j=0;j<topo.itotalPOINumber;j++)
			Psum+=p[Particle_No][k][j];
		for(j=0;j<topo.itotalPOINumber;j++)
			p[Particle_No][k][j]=p[Particle_No][k][j]/Psum;	
	}
}

LISTINT PsoOptimization::Generate_Candidate_POI(int NodeId_t,BASIC_ROUTE_Struct current_route,int Particle_No)
{
	int id=0,returnid=0;
	int minD_time=9999;
	LISTINT sv;
	LISTINT::iterator j;
	CRoute rt=CRoute();
	double rd=(double)rand() / RAND_MAX/topo.itotalPOINumber*2;
	double Tij=0,Psum=0,max=0;
	LISTPOI temppoi=topo.listPOI;
	LISTPOI::iterator i;
	rt.current_route=current_route;
	for(i=temppoi.begin();i!=temppoi.end();i++)
	{
		if((*i).id==NodeId_t)
		{}
		else
		{
			if(checkId((*i).id))
			{
				if(p[Particle_No][NodeId_t][(*i).id]>rd)
				{
					rt.InsertAfter(NodeId_t,(*i).id);
					if(Check_Buffer(rt.current_route))
					{
						//id=(*i).id;
						//f=current_topo.get_single_POI(NodeId_t).d_time;
						//l=current_topo.get_single_POI(id).d_time;
						//Tij=(double)(f-l);
						//Tij=current_topo.get_single_POI(NodeId_t).d_time;
						//Psum=Tij-current_topo.getDis(NodeId_t,id)/M_VELOCITY;
						sv.push_back((*i).id);
						if(p[Particle_No][NodeId_t][(*i).id]>max)
							max=p[Particle_No][NodeId_t][(*i).id];
						if((*i).d_time<minD_time)
						{
							minD_time=(*i).d_time;
							id=(*i).id;
						}
					}
					//LISTINT::iterator ite;
					//for(ite=rt.current_route.seq.begin();ite!=rt.current_route.seq.end();ite++)
					//cout<<*ite<<endl;
					rt.current_route.seq.remove((*i).id);
				}
			}
		}
	}
	//cout<<"1111111111"<<endl;
	//if(!sv.empty())
	//{
	//	for(j=sv.begin();j!=sv.end();j++)
	//	{
	//		if(p[Particle_No][NodeId_t][*j]>=max)
	//			id=*j;
	//	}
	//	sv.clear();
	//	sv.push_back(id);
	//}
	if(sv.empty())
	{
		for(i=temppoi.begin();i!=temppoi.end();i++)
		{
			if((*i).id==NodeId_t)
			{}
			else
			{
				if(checkId((*i).id))
				{
					//rt.InsertAfter(NodeId_t,(*i).id);
					if(Check_Buffer(rt.current_route))
					{
						sv.push_back((*i).id);
						if((*i).d_time<minD_time)
						{
							minD_time=(*i).d_time;
							id=(*i).id;
						}
					}
					rt.current_route.seq.remove((*i).id);
				}
			}
		}
	}
	if(!sv.empty())
	{
		sv.clear();
		sv.push_back(id);
	}
	////若零点是最大概率的返回点，则选取零点
	//if(p[Particle_No][NodeId_t][0]>max&&(NodeId_t!=0))
	//{
	//	sv.push_back(0);
	//}
	return sv;
}

CSolution PsoOptimization::Route_Building(int Particle_No)
{
	CSolution tmpsol;
	LISTINT sv,sx,sa;
	LISTINT::iterator iter;
	int t=0,k=0,i=0,s_num=1,isfeas=1,current_id=0;
	int sinkID=0;
	temPOI=topo.listPOI;
	sinkID=(*temPOI.begin()).id;//默认第一个点是sink节点
	temPOI.pop_front();
	t=sinkID;
	CRoute rt=CRoute();
	//tmpsol.routelist.clear();
	//tmpsol.route_num=0;
	//tmpsol.sensor_num=0;
	//Position[Particle_No].routelist.clear();
	//Position[Particle_No].route_num=0;
	//Position[Particle_No].sensor_num=0;
	while(!temPOI.empty())
	{
		if(k==0)
		{
			rt.current_route.seq.clear();
			rt.current_route.capacity=0;
			rt.current_route.len=0;
			rt.current_route.poi_num=0;
			rt.current_route.sensor_num=0;

			rt.current_route.seq.push_back(sinkID);
			rt.current_route.seq.push_back(sinkID);
			for(i=0;i<s_num;i++)
			{
				SENSOR_Struct sensor={BUFF_MAX,1,M_VELOCITY};
				rt.current_route.sensor_num++;
				rt.current_route.capacity=sensor.buffer;
			}
		}


		sv=Generate_Candidate_POI(t,rt.current_route,Particle_No);


		if(!sv.empty())
		{
			i=rand()%sv.size();
			//cout<<" "<<sv.size();
			//cout<<" rand i "<<i<<"  ";
			for(iter=sv.begin();iter!=sv.end();)
			{
				iter++;
				if(--i<0)
					break;
			}
			current_id=(*--iter);
			//cout<<current_id<<endl;
			if(current_id<0)
				cout<<"cuole"<<endl;
			if(current_id==0)
			{
				t=0;
				k=0;
				//rt.current_route=Two_Opt(rt.current_route);
				//cout<<"1234567890"<<endl;
			//	rt.CaclulateSensorNum();
			//	rt.recomLen();
				tmpsol.routelist.push_back(rt.current_route);
				isfeas=1;
			}
			else
			{
			//	rt.InsertAfter(t,current_id);
				t=k=current_id;
				isfeas=0;
				removePOI(current_id);
			}
		}
		else
		{
			t=0;
			k=0;
			//rt.current_route=Two_Opt(rt.current_route);2-opt
			//cout<<"9876543210"<<endl;
			//rt.CaclulateSensorNum();
		//	rt.recomLen();
			//recomLen(&rt.current_route);
			//recomSen_num(&rt.current_route);
			tmpsol.routelist.push_back(rt.current_route);
			isfeas=1;
		}


	}
	//rt.current_route=Two_Opt(rt.current_route);
	//rt.CaclulateSensorNum();
	//rt.recomLen();
	tmpsol.routelist.push_back(rt.current_route);
	
	tmpsol.LocalOptimize();//2-opt,优化
	tmpsol.route_num=tmpsol.routelist.size();
	//tmpsol.CalculateSensorNumber();
	tmpsol.List_to_RouteMatrix();


	//PrintSolution(Position[Particle_No]);
	if(!pSolution[Particle_No].routelist.empty())
	{
		if(tmpsol.sensor_num<=pSolution[Particle_No].sensor_num)
			pSolution[Particle_No]=tmpsol;
	}
	else
	{
		pSolution[Particle_No]=tmpsol;
	}
	//Position[Particle_No]=tmpsol;
	//PrintSolution(Position[Particle_No]);

	return pSolution[Particle_No];
}

bool PsoOptimization::checkId(int NodeId)
{
	LISTPOI::iterator j;
	//cout<<NodeId<<endl;
	for(j=temPOI.begin();j!=temPOI.end();j++)
	{
		if(NodeId==(*j).id)
			return true;
	}
	return false;
}

void PsoOptimization::removePOI(int NodeId)
{
	LISTPOI::iterator i;
	for(i=temPOI.begin();i!=temPOI.end();i++)
	{
		if((*i).id==NodeId)
		{
			if(NodeId==10)
				temPOI.erase(i);
			else
				temPOI.erase(i);
			return;
		}
	}
}

bool PsoOptimization::Check_Buffer(BASIC_ROUTE_Struct srs)
{
	LISTPOI::iterator i;
	LISTINT::iterator j;
	LISTINT list_nid;
	LISTPOI checkPOI;
	double buffer=0;
	list_nid=srs.seq;
	checkPOI=topo.listPOI;
	checkPOI.pop_front();
	for(j=(++list_nid.begin());j!=--list_nid.end();++j)//不考虑sink（0）点
	{
		for(i=checkPOI.begin();i!=checkPOI.end();++i)
		{
			if((*j)==(*i).id)
			{
				buffer+=(*i).demand;
			}
		}
	}
	if(buffer>srs.capacity)
	{
		return false;
	}
	return true;
}


void PsoOptimization::updateGbest()
{
	int i;
	CSolution temSol=CSolution();
	for(i=0;i<NumberofSolution;i++)
	{
		if(gBestSolution.sensor_num>pBestSolution[i].sensor_num)
		{
			gBestSolution=pBestSolution[i];
		}
		else
		{
			if(gBestSolution.sensor_num==pBestSolution[i].sensor_num)
				if(reComputeAllLength(gBestSolution)>reComputeAllLength(pBestSolution[i]))
				{
					gBestSolution=pBestSolution[i];
				}
		}
	}
}

void PsoOptimization::updatePbest()
{
	int i;
	CSolution temSol=CSolution();
	for(i=0;i<NumberofSolution;i++)
	{
		if(pBestSolution[i].sensor_num>pSolution[i].sensor_num)
		{
			pBestSolution[i]=pSolution[i];
		}
		else
		{
			if(pBestSolution[i].sensor_num==pSolution[i].sensor_num)
				if(reComputeAllLength(pBestSolution[i])>reComputeAllLength(pSolution[i]))
				{
					pBestSolution[i]=pSolution[i];
				}
		}
	}
}

double PsoOptimization::reComputeAllLength(CSolution cs)
{
	LISTROUTE::iterator i;
	double length=0;
	for(i=cs.routelist.begin();i!=cs.routelist.end();i++)
	{
		length+=(*i).len;
	}
	return length;
}