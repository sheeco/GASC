#include "Route.h"

extern CTopology topo;

CRoute::CRoute(void)
{
	//EDIT:应该初始化为BUFF_MAX
	current_route.capacity=BUFF_MAX;
	current_route.len=0;
	current_route.poi_num=0;
	current_route.sensor_num=0;
	current_route.seq.clear();
}

CRoute::CRoute(BASIC_ROUTE_Struct route)
{
	current_route=route;
	//EDIT:
	this->UpdateStatus();
}

CRoute::~CRoute(void)
{
	current_route.seq.clear();
}


bool CRoute::IsFeasible(int poi_id)
{
	LISTPOI::iterator i;
	LISTINT::iterator j;
	LISTPOI checkPOI;
	double demand=0;
	checkPOI=topo.listPOI;
	for(i=checkPOI.begin();i!=checkPOI.end();++i)
	{
		if(poi_id==(*i).id)
		{
			demand=(*i).demand;
			break;
		}
	}

	if(current_route.capacity>=demand)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CRoute::IsFeasibleTimeAndBuffer(int poi_id)
{
	LISTPOI::iterator i;
	LISTINT::iterator j;
	double Interval=0;
	LISTINT list_nid;
	LISTPOI checkPOI;
	double buffer=0;
	double minLen=9999;
	double delta=0;
	LISTINT::iterator m,n,pos;
	checkPOI=topo.listPOI;
	if(IsFeasible(poi_id))//检测buffer是否可容
	{
		list_nid=current_route.seq;

	
		for(m=list_nid.begin();m!=--list_nid.end();m++)
		{
			n=m;
			n++;
			delta=topo.GetDistance((*m),poi_id)+topo.GetDistance(poi_id,(*n))-topo.GetDistance((*m),(*n));
			if(minLen>delta)
			{
				minLen=delta;
				pos=m;
			}
		}
	
		list_nid.insert(++pos,poi_id);

		minLen=0;
		for(m=(list_nid.begin());m!=--list_nid.end();++m)
		{
			n=m;
			n++;
			minLen+=topo.GetDistance((*m),(*n));
		}

		Interval=minLen;
		Interval=Interval/(current_route.sensor_num *M_VELOCITY);//时间间隔
		for(j=(++list_nid.begin());j!=--list_nid.end();++j)
		{
			for(i=checkPOI.begin();i!=checkPOI.end();++i)
			{
				if((*j)==(*i).id)
				{
					if((*i).d_time<Interval)//检测覆盖周期是否满足
						return false;
				}
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}

void CRoute::UpdateStatus()
{

	//更新长度
	//更新capacity
	//更新poi_num
	//更新移动sensor数量
	LISTPOI::iterator i;
	LISTINT::iterator j,m,n;
	LISTINT list_nid;
	LISTPOI checkPOI;
	double buffer=0;
	double length=0;
	int min_time=9999;
	list_nid=current_route.seq;

	n=m=list_nid.begin();
	n++;
	length+=topo.GetDistance((*m),(*n));
	checkPOI=topo.listPOI;
	for(j=(++list_nid.begin());j!=--list_nid.end();++j)//这里把sink给去掉了
	{

		n=m=j;
		n++;
		length+=topo.GetDistance((*m),(*n));

		for(i=checkPOI.begin();i!=checkPOI.end();++i)
		{
			if((*j)==(*i).id)
			{
				buffer+=(*i).demand;

				if(min_time>(*i).d_time)
					min_time=(*i).d_time;

				break;
			}
		}

	}
	current_route.len=length;
	current_route.capacity=BUFF_MAX-buffer;
	current_route.poi_num=current_route.seq.size()-2;//去掉首尾两个sink节点
	current_route.sensor_num=(int)ceil((current_route.len/M_VELOCITY)/(0+(double)min_time));//[L/(V*D_Min)]向上取整
}

bool CRoute::InsertAfter(int pos,int id)
{
	bool r_value;
	LISTINT::iterator i;
	LISTPOI::iterator j;
	LISTPOI templist=topo.listPOI;
	if((i=find(current_route.seq.begin(),current_route.seq.end(),pos))==current_route.seq.end())
		r_value=false;
	else
	{
		if(find(current_route.seq.begin(),current_route.seq.end(),id)!=current_route.seq.end())
			r_value=false;
		else
		{
			if(IsFeasible(id))
			{
				current_route.seq.insert(++i,id);
				UpdateStatus();
				r_value=true;
			}
			else
			{
				r_value=false;
			}
			
		}
	}
	return r_value;
}

bool CRoute::DeleteFromRoute(int id)
{
	bool r_value;
	LISTINT::iterator i;
	if((i=find(current_route.seq.begin(),current_route.seq.end(),id))==current_route.seq.end())
		r_value=false;
	else
	{
		current_route.seq.erase(i);
		UpdateStatus();
		r_value=true;
	}
	return r_value;
}

bool CRoute::SolomenInsert(int id)
{
	double minLen=9999;
	double delta=0;
	LISTINT::iterator i,j;
	int pos;
	for(i=current_route.seq.begin();i!=--current_route.seq.end();i++)
	{
		j=i;
		j++;
		delta=topo.GetDistance((*i),id)+topo.GetDistance(id,(*j))-topo.GetDistance((*i),(*j));
		if(minLen>delta)
		{
			minLen=delta;
			pos=(*i);
		}
	}
	return InsertAfter(pos,id);
}

bool CRoute::Is_Empty()
{
	if(current_route.seq.size()<=2)
	{
		if(current_route.seq.size()==2)
		{
			if(current_route.seq.front()==0 && current_route.seq.back()==0)
				return true;
		}
		//cout<<"路径出错了："<<endl;
	//	PrintRoute();//打印出错路径
		return false;
	}
	else
	{
		return false;
	}
}

void CRoute::PrintRoute()
{
	LISTINT::iterator it;
	for(it=current_route.seq.begin();it!=current_route.seq.end();it++)
	{
		cout<<(*it)<<" ";
	}
	cout<<endl;
}

BASIC_ROUTE_Struct CRoute::Two_Opt(BASIC_ROUTE_Struct*brs)
{
	double old_edge_dists ,new_edge_dists ,improvement,best_improvement = 0;
	LISTINT::iterator li,lj,fi,nj,best_i, best_j,iplus;
	int count=0;
	BASIC_ROUTE_Struct new_brs;
	int sinkID=topo.get_single_POI(0).id;
	if(brs->seq.size()<=3)
		return *brs;
	for(li=brs->seq.begin();li!=----brs->seq.end();li++)//事实上交换的是li和lj两个点!
	{
		fi=li;
		if(fi!=brs->seq.begin())
		{
			fi--;
		}
		else
		{
			fi=----brs->seq.end();
		}
		iplus=li;
		iplus++;
		for(lj=iplus;lj!=--brs->seq.end();lj++)
		{
			if(li==brs->seq.begin()&&lj==----brs->seq.end())
			{
				fi=li;
				fi++;
				nj=lj;
				nj--;
				old_edge_dists=topo.GetDistance(*fi,*li)+topo.GetDistance(*lj,*nj);
				new_edge_dists=topo.GetDistance(*nj,*li)+topo.GetDistance(*lj,*fi);
			}
			else
			{
				nj=lj;
				nj++;
				old_edge_dists=topo.GetDistance(*fi,*li)+topo.GetDistance(*lj,*nj);
				new_edge_dists=topo.GetDistance(*fi,*lj)+topo.GetDistance(*li,*nj);
			}
			improvement = old_edge_dists - new_edge_dists;
			if(improvement>best_improvement)
			{
				best_improvement = improvement;
				best_i = li;
				best_j = lj;
			}
		}
	}
	if(best_improvement>0)
	{ 

		li=brs->seq.begin();
		while (li!=best_i)
		{
			new_brs.seq.push_back(*li);
			count++;
			li++;
		}
		if(li==best_i)
		{
			li=best_j;
			new_brs.seq.push_back(*li);
			count++;
			li--;
			while (li!=best_i)
			{
				new_brs.seq.push_back(*li);
				li--;
				count++;
			}
			new_brs.seq.push_back(*li);
			count++;
			li=best_j;
		}
		li++;
		while (li!=--brs->seq.end())
		{
			new_brs.seq.push_back(*li);
			count++;
			li++;
		}
		li=new_brs.seq.begin();
		brs->seq.clear();
		while (*li!=sinkID)
		{
			li++;
		}
		while(li!=new_brs.seq.end())
		{
			brs->seq.push_back(*li);
			li++;
		}
		li=new_brs.seq.begin();
		while (*li!=sinkID)
		{
			brs->seq.push_back(*li);
			li++;
		}
		brs->seq.push_back(sinkID);
		CRoute cr=CRoute();
		cr.current_route=*brs;
		cr.UpdateStatus();
		//recomLen(&brs);
		//brs.sensor_num=recomSen_num(&brs);
		return cr.current_route;
	}
	else
	{
		return *brs;
	}
}

