#include "GA_Solution.h"

extern CTopology topo;

CGA_Solution::CGA_Solution(void)
{
	this->fitnessvalue = 0;
	this->NumberofUnImproved = 0;
	this->localbest=0;
}

CGA_Solution::CGA_Solution(const CSolution& it)
{
	*this = it;
	this->fitnessvalue = 0;
	this->NumberofUnImproved = 0;
	this->localbest=0;
	//this->birth = 0;
}

CGA_Solution::~CGA_Solution(void)
{
}

//���ز�����=
CGA_Solution& CGA_Solution::operator=(const CGA_Solution& it)
{
	this->fitnessvalue = it.fitnessvalue;
	this->NumberofUnImproved=it.NumberofUnImproved;
	this->routelist=it.routelist;
	this->route_num=it.route_num;
	this->sensor_num=it.sensor_num;
	this->totallength=it.totallength;
	this->localbest=it.localbest;

	int i,j;
	for(i=0;i<topo.itotalPOINumber;i++)
	{
		for(j=0;j<topo.itotalPOINumber;j++)
		{
			this->pRouteMatrix[i][j]=0;
			this->pRouteMatrix[i][j]=it.pRouteMatrix[i][j];
		}
	}

	return *this;
}

//FIXME:Ŀǰ�����·����Ŀ��POI��Ŀ����·������ͬ����Ϊ����������ͬ
bool CGA_Solution::SameAs(/*const*/ CGA_Solution& it)
{
	if(this->totallength != it.totallength)
		return false;
	else if(this->route_num != it.route_num
		|| this->sensor_num != it.sensor_num)
		return false;
	else
		return true;

 }

//�ж�һ��POI�Ƿ��ѱ���������·���У�����true��false��route_indexΪPOIλ�ڵ�·���±�
bool CGA_Solution::IfPOIExists(int poi_seq, int &route_index)
{
	if(poi_seq == 0)
		return false;
	route_index = -1;
	LISTROUTE::iterator curr_route =  routelist.begin();
	int curr_route_index;
	LISTINT::iterator curr_poi;
	for(curr_route_index = 0; curr_route != routelist.end(); curr_route++, curr_route_index++)
	{
		curr_poi = curr_route->seq.begin();
		for(; curr_poi != curr_route->seq.end(); curr_poi++)
		{
			if(*curr_poi == poi_seq)
			{
				route_index = curr_route_index;
				return true;
			}
		}
	}
	return false;
}

double CGA_Solution::getFitnessValue()
{
	if(sensor_num == 0)
		return 0;

	fitnessvalue = 100/(RATE_SENSOR_NUM * sensor_num + RATE_DISTANCE * totallength);
	return fitnessvalue;
}

void CGA_Solution::setFitnessValue(double fitness)
{
	this->fitnessvalue = fitness;
}

bool CGA_Solution::IsValid()
{
	for(LISTROUTE::iterator it = routelist.begin();it !=routelist.end(); )
	{
		if(CRoute(*it).Is_Empty())
		{
			it = routelist.erase(it);
			//system("pause");
			//return false;
		}
		else
			it++;
	}
	return true;
}

bool CGA_Solution::DeletePOI(int seq)
{
	int i;
	int selected_route_index;
	LISTROUTE::iterator selected_route;
	if(seq == 0)
		return false;
	if(this->IfPOIExists(seq, selected_route_index))
	{
		for(i = 0, selected_route = this->routelist.begin(); i < selected_route_index; i++)
			selected_route++;
		CRoute new_route(*selected_route);
		new_route.DeleteFromRoute(seq);
		for(i = 0, selected_route = this->routelist.begin(); i < selected_route_index; i++)
			selected_route++;
		*selected_route = new_route.current_route;
	}
	return true;
}

bool CGA_Solution::DeletePOIList(LISTINT seqlist)
{
	LISTINT::iterator seq;
	for(seq = seqlist.begin(); seq != seqlist.end(); seq++)
	{
		this->DeletePOI(*seq);
	}
	return true;
}

bool CGA_Solution::ReInsertPOIList(LISTINT poi_list)
{
	LISTINT::iterator selected_poi;
	int route_index;
	for(selected_poi = poi_list.begin(); selected_poi != poi_list.end(); selected_poi++)
	{
		if(*selected_poi == 0)
			continue;
		if(this->IfPOIExists(*selected_poi, route_index))
			this->DeletePOI(*selected_poi);
		if(! Insert_Best_Route(*this, *selected_poi))
			return false;
	}
	this->UpdateStatus();
	return true;
}

//ɾ���ظ��㣬����·���е�POI�ٲ��뵽��ǰ�⣨�������Ľ⣩��
bool CGA_Solution::ReInsertRouteToSolution(BASIC_ROUTE_Struct route)
{
	LISTINT poi_list = route.seq;  //�������POI��������sink
	LISTINT::iterator selected_poi;
	LISTROUTE::iterator selected_route;
	//ɾ���ظ���POI
	this->DeletePOIList(poi_list);
	this->UpdateStatus();
	this->ReInsertPOIList(poi_list);
	return true;
}


//���������⼰����ÿ��·������Ratio = POI_num/sensor_num
//RatioԽ�͵ĵ㱻ѡ�еĸ���Խ��
void CGA_Solution::SelectRouteByRatio(int &route_n)  //ѡ��һ��·��
{
	double ratio_route = 0;
	double ratio_solution = 0;
	LISTROUTE::iterator iroute;
	vector<int> v;
	int i;
	this->UpdateStatus();
	ratio_solution = (float)(this->GetPOINumber()) / sensor_num;
	//�ҳ�����ratio��������ֵ��·��
//DBG:
	//cout<<"\nratios: "<<ratio_solution<<" | ";
	for(iroute = this->routelist.begin(), i = 0; iroute != this->routelist.end(); iroute++, i++)
	{
		ratio_route = (float)(iroute->poi_num) / iroute->sensor_num;
//DBG:
		//cout<<ratio_route<<"  ";
		if(ratio_route <= ratio_solution)
			v.push_back(i);
	}
	
	//���ѡ��һ��
	int bet;
	if(v.empty())
		system("pause");
	bet = RandomInt(0, v.size());
	route_n = v.at(bet);
	//cout<<"| choose "<<route_n<<endl;
}

void CGA_Solution::SelectRouteByRatio(int &route1_n, int &route2_n)  //ѡ������·��
{
	int n1 = 0;
	int n2 = 0;
	double ratio_route = 0;
	double ratio_solution = 0;
	LISTROUTE::iterator iroute;
	vector<int> v;
	int i;
	this->UpdateStatus();
	ratio_solution = (float)(this->GetPOINumber()) / sensor_num;
	//�ҳ�����ratio��������ֵ��·��
	for(iroute = this->routelist.begin(), i = 0; iroute != this->routelist.end(); iroute++, i++)
	{
		ratio_route = (float)(iroute->poi_num) / iroute->sensor_num;
		if(ratio_route <= ratio_solution)
			v.push_back(i);
	}
	//�������������·��ֻ��һ�������ҳ�����·����ratio��͵�һ��
	double leastRatio = 0;
	int tmp_index;
	if(v.size() == 1)
	{
		for(iroute = this->routelist.begin(), i = 0; iroute != this->routelist.end(); iroute++, i++)
		{
			if(i == v.at(0))
				continue;
			ratio_route = (float)(iroute->poi_num) / iroute->sensor_num;
			if(leastRatio == 0)
			{
				leastRatio = ratio_route;
				tmp_index = i;
			}
			else if(leastRatio > ratio_route)
			{
				leastRatio = ratio_route;
				tmp_index = i;
			}
		}
		v.push_back(tmp_index);
	}
	//���ѡ����������ͬ��·��
	int bet;
	vector<int>::iterator iindex;
	bet = RandomInt(0, v.size());
	n1 = v.at(bet);
	for(i = 0, iindex = v.begin(); i < bet;i++)
		iindex++;
	v.erase(iindex);
	bet = RandomInt(0, v.size());
	n2 = v.at(bet);
	route1_n = n1;
	route2_n = n2;
}

void CGA_Solution::UpdateStatus()
{
	this->IsValid();
	CSolution::UpdateStatus();
	getFitnessValue();
}

void CGA_Solution::PrintSolution()
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
		{
			cout<<*j<<" ";
			savetofile<<*j<<" ";
		}
		cout<<" the sensor number of this route is "<<(*i).sensor_num<<endl;
		savetofile<<" the sensor number of this route is "<<(*i).sensor_num<<endl;
		poi_num=poi_num+((*i).seq.size()-2);
		all_len+=(*i).len;
		s_num1+=(*i).sensor_num;
	}
	sensor_num=s_num1;
	route_num=routelist.size();
	cout<<"sol.route_num     "<<route_num<<endl;
	cout<<"sol.sensor_num    "<<sensor_num<<endl;
	cout<<"sol.length        "<<all_len<<endl;
	cout<<"sol.poi_num       "<<poi_num<<endl;
	cout<<"sol.fitnessvalue  "<<getFitnessValue()<<endl;
	savetofile<<"sol.route_num     "<<route_num<<endl;
	savetofile<<"sol.sensor_num    "<<sensor_num<<endl;
	savetofile<<"sol.length        "<<all_len<<endl;
	savetofile<<"sol.poi_num       "<<poi_num<<endl;
	savetofile<<"sol.fitnessvalue  "<<getFitnessValue()<<endl;
}
