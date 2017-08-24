#include "GA.h"

int debug_int = 250;

CGA::CGA(void)
{
	this->NumberofSolution = POPULATION_SIZE;
	this->pSolution.clear();
	for (int i = 0; i < POPULATION_SIZE; i++)
	{
		this->pSolution.push_back(CGA_Solution());
	}
	this->BestSolution = CGA_Solution();
	this->NumberofUnImproved = 0;
	this->BestUnImproved = 0;
	this->current_op1 = 0;
	this->current_op2 = 0;
	this->runninground=0;
	this->worst_first = 0;
	this->worst_second = 0;
	this->sumFitnessValue = 0;
}

CGA::~CGA(void)
{
	vector<CGA_Solution> tmp;
	pSolution.swap(tmp);
}

//初始解产生，由main函数调用
void CGA::GenerateInit(void)
{
	unsigned int i;
	unsigned int temp=65535;
//DBG:
	cout<<"初始解生成中..."<<endl;

		for (i=0;i<pSolution.size();i++)
	{
		pSolution[i].GetRandomSolution();
		if(pSolution[i].sensor_num<temp)
		{
			pSolution[i].localbest=pSolution[i].sensor_num;			
			temp=pSolution[i].sensor_num;
			BestSolution=pSolution[i];
		}
	}
//DBG:
	cout<<"初始解生成完成！"<<endl;

	FindBestSolution();
	FindWorstSolution();
}

//优化函数，控制主要过程，由main函数调用
CSolution CGA::Optimize(void)
{
	double bet1 = 0;
	int bet2 = 0;
	bool performCrossover = false;  //是否实施交叉
	bool performMutation = false;  //是否实施变异
	bool improved = false;  //是否更新种群
	CGA_Solution *output1, *output2, *output3, *output4;  //用于保存交叉、变异后的结果

	while(!OptCompleted())//优化未达到临界条件
	{
		cout<<"************************************************************************"<<endl;
		cout<<"Optimization "<< runninground <<" begins."<<endl;
		if(runninground % debug_int == 0)
		{
			savetofile<<"************************************************************************"<<endl;
			savetofile<<"Optimization "<< runninground <<endl;
		}
//DBG:
		//cout<<"pSolution.size: "<<pSolution.size()<<endl;
		//if(pSolution.size() == 0)
		//	system("pause");
		//cout<<"Fitness Values:\n";
		//for(int i = 0; i< pSolution.size(); i++)
		//	cout<< pSolution[i].getFitnessValue()<<" | ";

		output1 = new CGA_Solution();
		output2 = new CGA_Solution();
		output3 = new CGA_Solution();
		output4 = new CGA_Solution();

		//轮盘赌选择两个个体
		SelectTwo();
//DBG:
		//printf("Selected indivs: %d, %d\n", current_op1, current_op2);
		//pSolution[current_op1].PrintSolution();
		//pSolution[current_op2].PrintSolution();

		bet1 = RandomFloat(0, 1);

//DBG:
		//cout<<"bet1: "<<bet1<<endl;

		//按交叉概率实施交叉
		if(bet1 <= PROB_CROSSOVER)
		{
			//随机实施三种交叉算子中的一种
			bet2 = RandomInt(0, 3);
//DBG:
			//cout<<"bet2 : "<<bet2<<endl;

			switch(bet2)
			{
			case 0:
				//cout<<"对称匹配交叉"<<endl;
				SymmCrossover(pSolution[current_op1], pSolution[current_op2], *output1, *output2);
				break;
			case 1:
				//cout<<"非对称匹配交叉"<<endl;
				UnSymmCrossover(pSolution[current_op1], pSolution[current_op2], *output1, *output2);
				break;
			case 2:
				//cout<<"线路交叉"<<endl;
				RouteCrossover(pSolution[current_op1], pSolution[current_op2], *output1, *output2);
				break;
			default:
				break;
			}

//DBG:
			//printf("Output1:\n");
			//output1->PrintSolution();
			//printf("Output2:\n");
			//output2->PrintSolution();

		}//end if performing crossover
		//如果不实施交叉，直接复制被选中的两个个体，用于实施变异
		else
		{
//DBG:
			cout<<"不实施交叉        ";

			*output1 = pSolution[current_op1];
			*output2 = pSolution[current_op2];
		}

		//按变异概率实施变异
		bet1 = RandomFloat(0, 1);
//DBG:
		//cout<<"bet1: "<<bet1<<endl;

		if(bet1 <= PROB_MUTATION)
		{
			//随机实施两种变异算子中的一种
			bet2 = RandomInt(0, 2);
//DBG:
			//cout<<"bet2 : "<<bet2<<endl;

			switch (bet2)
			{
			case 0:
				RouteMutution(*output1, *output3);
				RouteMutution(*output2, *output4);
				break;
			case 1:
				POIMutution(*output1, *output3);
				POIMutution(*output2, *output4);
				break;
			default:
				break;
			}

//DBG:

			//printf("output3: \n");
			//output3->PrintSolution();
			//printf("output4: \n");
			//output4->PrintSolution();

		}//end if performing mutation
		//如果不实施变异，直接复制output1, output2作为本次迭代的结果
		else
		{
//DBG:
			cout<<"		不实施变异\n";

			*output3 = *output1;
			*output4 = *output2;
		}

		/*如果output3, output4与原种群中染色体不相同，
		  且适应度值好于或等于原种群中最差的个体，则进行更新*/
		FindWorstSolution();
		int i;

		LISTSOLUTION::iterator replaced;
		if(output3->getFitnessValue() >= pSolution[worst_first].getFitnessValue()
			&& !IfExists(*output3) )
		{
			for(i = 0, replaced = pSolution.begin(); i < worst_second; i++)
				replaced++;
			replaced = pSolution.erase(replaced);
			pSolution.insert(replaced, *output3);
			if(output3->getFitnessValue() > pSolution[worst_first].getFitnessValue())
				improved = true;
//DBG:
			//printf("Output3 replace indiv %d, ", worst_second);
		}
		if(output4->getFitnessValue() >= pSolution[worst_first].getFitnessValue()
			&& !IfExists(*output4) )
		{
			for(i = 0, replaced = pSolution.begin(); i < worst_first; i++)
				replaced++;
			replaced = pSolution.erase(replaced);
			pSolution.insert(replaced, *output4);
			
			if(output4->getFitnessValue() > pSolution[worst_first].getFitnessValue())
				improved = true;
//DBG:
			//printf("Output4 replace indiv %d\n", worst_first);
		}

		NumberofSolution += 2;
		if(improved)
			NumberofUnImproved = 0;
		else
			NumberofUnImproved++;
		FindBestSolution();
//DBG:
		cout<<"\nBest Solution Now: "<<endl;
		cout<<"route_num: "<<BestSolution.route_num;
		cout<<", sensor_num: "<<BestSolution.sensor_num;
		cout<<", length: "<<BestSolution.totallength;
		cout<<", fitness: "<<BestSolution.getFitnessValue()<<endl;
		if(runninground % debug_int == 0)
		{
			savetofile<<"\nBest Solution Now: "<<endl;
			savetofile<<"route_num: "<<BestSolution.route_num<<endl;
			savetofile<<"sensor_num: "<<BestSolution.sensor_num<<endl;
			savetofile<<"length: "<<BestSolution.totallength<<endl;
			savetofile<<"fitness: "<<BestSolution.getFitnessValue()<<endl;
		}
		performCrossover = false;
		performMutation = false;
		improved = false;
		delete output1;
		delete output2;
		delete output3;
		delete output4;

		runninground++; 
	}
	//优化结束，打印最终结果
	finalstatus();
	return BestSolution;
}

/*几种交叉、变异算子*/
//对称匹配交叉
bool CGA::SymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2)
{
	output1 = indiv1;
	output2 = indiv2;

	//FIXME:如果要进行多条线路的对称交叉，更改crossover_num即可（多次选择的线路号可能重复！）
	//进行交叉的路线的数量
	int crossover_num = 1;
	int i, j;
	int selected_route_index1, selected_route_index2;
	LISTROUTE::iterator curr_route1, curr_route2;
	
	for(j = 0; j < crossover_num; j++)
	{
//RATIO:
		//根据Ratio选择两条路径
		output1.SelectRouteByRatio(selected_route_index1);
		output2.SelectRouteByRatio(selected_route_index2);
		//随机选择一对路径
		//selected_route_index1 = RandomInt(0, output1.routelist.size());
		//selected_route_index2 = RandomInt(0, output2.routelist.size());
		//删除被选中的两条路径，保留副本
		for(i = 0, curr_route1 = output1.routelist.begin(); i < selected_route_index1 && curr_route1!= output1.routelist.end(); i++)
			curr_route1++;
		BASIC_ROUTE_Struct selected_route1(*curr_route1);
		curr_route1 = output1.routelist.erase(curr_route1);
		output1.UpdateStatus();
		for(i = 0, curr_route2 = output2.routelist.begin(); i < selected_route_index2 && curr_route2!= output2.routelist.end(); i++)
			curr_route2++;
		BASIC_ROUTE_Struct selected_route2(*curr_route2);
		curr_route2 = output2.routelist.erase(curr_route2);
		output2.UpdateStatus();
		//先删除将要交叉的路径中已存在的POI
		output1.DeletePOIList(selected_route2.seq);
		output2.DeletePOIList(selected_route1.seq);
		//交换线路
		output1.routelist.insert(curr_route1, selected_route2);
		output2.routelist.insert(curr_route2, selected_route1);
		//收集解中缺少的POI，进行再插入
		LISTINT reinsert_poi_list1, reinsert_poi_list2;
		int tmp;
		for(i = 1; i < topo.itotalPOINumber; i++)
		{
			if(!output1.IfPOIExists(i, tmp))
			{
				reinsert_poi_list1.push_back(i);
			}
			if(!output2.IfPOIExists(i, tmp))
			{
				reinsert_poi_list2.push_back(i);
			}
		}
		output1.ReInsertPOIList(reinsert_poi_list1);
		output2.ReInsertPOIList(reinsert_poi_list2);
	}
	output1.UpdateStatus();
	output2.UpdateStatus();

	cout<<"对称匹配交叉完成";
	return true;
}

//非对称匹配交叉
bool CGA::UnSymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2)
{
	output1 = indiv1;
	output2 = indiv2;

	//FIXME:如果要求非对称交叉的线路数量随机决定，更改crossover_num1, crossover_num1的初始化
	//进行交叉的路线的数量：从output1中选择num1条线路，从output2中选择num2条线路
	int crossover_num1, crossover_num2;
	crossover_num1 = 1;
	crossover_num2 = 2;
	int i, j, k;
	//FIXME:如果要求非对称交叉的线路数量随机决定，此处可能要更改
	int selected_route_index1[1];
	int selected_route_index2[2];
	LISTROUTE::iterator curr_route1, curr_route2;
	BASIC_ROUTE_Struct selected_route1[1];
	BASIC_ROUTE_Struct selected_route2[2];
	
	//随机选择待交叉的路径
	for(j = 0; j < crossover_num1; j++)
	{
//RATIO:
		output1.SelectRouteByRatio(selected_route_index1[j]);
		//selected_route_index1[j] = RandomInt(0, output1.routelist.size());
		for(i = 0; i < crossover_num1 - 1; i++)
		{
			if(selected_route_index1[i] == selected_route_index1[j])
			{	
				j--;
				break;
			}
		}
	}
//RATIO:
	output2.SelectRouteByRatio(selected_route_index2[0], selected_route_index2[1]);
	//selected_route_index2[0] = RandomInt(0, output2.routelist.size());
	//do
	//{
	//	selected_route_index2[1] = RandomInt(0, output2.routelist.size());
	//}while(selected_route_index2[1] == selected_route_index2[0]);

	////selected_route_index2[1] = RandomInt(0, output2.routelist.size());
	////if(selected_route_index2[1] == selected_route_index2[0])
	////	selected_route_index2[1] = output2.routelist.size() - 1 - selected_route_index2[0];
	////if(selected_route_index2[1] == selected_route_index2[0])
	////	selected_route_index2[1]++;

	//先删除下标较小的路径
	if(selected_route_index2[0] < selected_route_index2[1])
	{
		int tmp;
		tmp = selected_route_index2[0];
		selected_route_index2[0] = selected_route_index2[1];
		selected_route_index2[1] = tmp;
	}

	//删除被选中的路径，保留副本
	for(j = 0; j < crossover_num1; j++)
	{
		if(selected_route_index1[j] == output1.routelist.size())
			selected_route_index1[j]--;
		for(i = 0, curr_route1 = output1.routelist.begin();
			(i < selected_route_index1[j]) && (curr_route1 != output1.routelist.end()); i++)
			curr_route1++;
		selected_route1[j] = *curr_route1;
		curr_route1 = output1.routelist.erase(curr_route1);
		output1.UpdateStatus();
	}
	for(j = 0; j < crossover_num2; j++)
	{
		if(selected_route_index2[j] == output2.routelist.size())
			selected_route_index2[j]--;
		for(i = 0, curr_route2 = output2.routelist.begin();
			(i < selected_route_index2[j]) && (curr_route2 != output2.routelist.end()); i++)
			curr_route2++;
		selected_route2[j]= *curr_route2;
		curr_route2 = output2.routelist.erase(curr_route2);
		output2.UpdateStatus();
	}

	//先删除将要交叉的路径中已存在的POI
	for(k = 0; k < crossover_num2; k++)
		output1.DeletePOIList(selected_route2[k].seq);
	for(k = 0; k < crossover_num1; k++)
		output2.DeletePOIList(selected_route1[k].seq);
	//交换线路
	for(k = 0; k < crossover_num2; k++)
		output1.routelist.insert(curr_route1, selected_route2[k]);
	for(k = 0; k < crossover_num1; k++)
		output2.routelist.insert(curr_route2, selected_route1[k]);
	//收集解中缺少的POI，进行再插入
	LISTINT reinsert_poi_list1, reinsert_poi_list2;
	int tmp;
	for(i = 1; i < topo.itotalPOINumber; i++)
	{
		if(!output1.IfPOIExists(i, tmp))
		{
			reinsert_poi_list1.push_back(i);
		}
		if(!output2.IfPOIExists(i, tmp))
		{
			reinsert_poi_list2.push_back(i);
		}
	}
	output1.ReInsertPOIList(reinsert_poi_list1);
	output2.ReInsertPOIList(reinsert_poi_list2);

	cout<<"非对称匹配交叉完成";
	return true;
}

//线路交叉
bool CGA::RouteCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2)
{
	output1 = indiv1;
	output2 = indiv2;

	int i;
	int selected_route_index1, selected_route_index2;
	LISTROUTE::iterator curr_route1, curr_route2;
//RATIO:
	output1.SelectRouteByRatio(selected_route_index1);
	output2.SelectRouteByRatio(selected_route_index2);
	//随机选择一对路径
	//selected_route_index1 = RandomInt(0, output1.routelist.size());
	//selected_route_index2 = RandomInt(0, output2.routelist.size());
	for(i = 0, curr_route1 = output1.routelist.begin(); i < selected_route_index1; i++)
		curr_route1++;
	BASIC_ROUTE_Struct selected_route1(*curr_route1);
	for(i = 0, curr_route2 = output2.routelist.begin(); i < selected_route_index2; i++)
		curr_route2++;
	BASIC_ROUTE_Struct selected_route2(*curr_route2);

//DBG:
	//LISTINT::iterator tmp;
	//printf("Selected routes:\n");
	//for(tmp = selected_route1.seq.begin(); tmp != selected_route1.seq.end(); tmp++)
	//	printf("%d ", *tmp);
	//printf("\n");
	//for(tmp = selected_route2.seq.begin(); tmp != selected_route2.seq.end(); tmp++)
	//	printf("%d ", *tmp);
	//printf("\n");

	//去重，交叉再插入
	if(!output1.ReInsertPOIList(selected_route2.seq))
		return false;
	if(!output2.ReInsertPOIList(selected_route1.seq))
		return false;

	//else 
	{
		cout<<"线路交叉完成      ";
		return true;
	}
}

//线路变异(R1算子)
bool CGA::RouteMutution(CGA_Solution &indiv1, CGA_Solution &output1)
{
	output1 = indiv1;

	int i;
	int selected_route_index;
	LISTROUTE::iterator curr_route;
//RATIO:
	output1.SelectRouteByRatio(selected_route_index);
	//随机选择一条路径
	//selected_route_index = RandomInt(0, output1.routelist.size());
	//删除被选中的路径
	for(i = 0, curr_route = output1.routelist.begin(); i < selected_route_index && curr_route != output1.routelist.end(); i++)
	{
		curr_route++;
	}
	BASIC_ROUTE_Struct selected_route;
	selected_route = *curr_route;
	output1.routelist.erase(curr_route);
	output1.UpdateStatus();
	//被删除的POI集再插入到原路径中去
//DBG:
	//printf("**ReInsert pois: \n");
	//LISTINT::iterator tmp;
	//for(tmp = selected_route.seq.begin(); tmp != selected_route.seq.end(); tmp++)
	//	printf("%d ", *tmp);
	//printf("\n");

	if(!output1.ReInsertPOIList(selected_route.seq))
		return false;
	else 
	{
		cout<<"		线路变异完成";
		return true;
	}
}

//单点变异(R2算子)
bool CGA::POIMutution(CGA_Solution &indiv1, CGA_Solution &output1)
{
	output1 = indiv1;

	int i;
	int selected_poi_seq;
	int selected_route_index;
	LISTROUTE::iterator curr_route;
	//随机选择一个POI
	selected_poi_seq = RandomInt(1, topo.itotalPOINumber);
	if(output1.IfPOIExists(selected_poi_seq, selected_route_index))
	{
		for(i = 0, curr_route = output1.routelist.begin(); i < selected_route_index && curr_route != output1.routelist.end(); i++)
			curr_route++;
		CRoute new_route(*curr_route);
		//将选中的POI重新插入到原路径当中
		new_route.DeleteFromRoute(selected_poi_seq);
		new_route.SolomenInsert(selected_poi_seq);
		output1 = output1;
		for(i = 0, curr_route = output1.routelist.begin(); i < selected_route_index && curr_route != output1.routelist.end(); i++)
			curr_route++;
		*curr_route = new_route.current_route;
		output1.UpdateStatus();
		cout<<"		单点变异完成";
		return true;
	}
	else
		return false;
}

/*Supportive Functions*/

//按轮盘赌选择两个个体，并将 <var>current_op1</var> 和 <var>current_op2</var> 指向它们
//如果成功，返回true
bool CGA::SelectTwo()
{
	double bet1, bet2;
	double curr = 0;
	int pos1 = -1, pos2 = -1;
	unsigned int i, j;
	int begin_pos = 0;
	
	bet1 = (double)RandomFloat(0, sumFitnessValue);
	begin_pos = RandomInt(0, pSolution.size());
//DBG:
	//cout<<"SelectTwo::sumFitnessValue = "<<sumFitnessValue<<endl;
	//cout<<"SelectTwo::bet1 = "<<bet1<<endl;

	for(i = begin_pos, j = 0; j < pSolution.size(); j++, i = (i+1)%pSolution.size())
	{
		curr += pSolution[i].getFitnessValue()/BestSolution.getFitnessValue()/10;
		if(curr >= bet1)
		{
			pos1 = i;
			break;
		}
	}
	if(pos1 == -1)
	{
		pos1 = (begin_pos - 1 + pSolution.size()) % pSolution.size();
	}
	do
	{
		curr = 0;
		bet2 = (double)RandomFloat(0, sumFitnessValue);
		begin_pos = RandomInt(0, pSolution.size());
		for(i = begin_pos, j = 0; j < pSolution.size(); j++, i = (i+1)%pSolution.size())
		{
			curr += pSolution[i].getFitnessValue()/BestSolution.getFitnessValue()/10;
			if(curr >= bet2)
			{
				pos2 = i;
				break;
			}
		}
	}while(pos1 == pos2 || pos2 == -1);
	if(pos1 != -1 && pos2 != -1)
	{
		current_op1 = pos1;
		current_op2 = pos2;
	}
	else
	{
		cout<<"pos1: "<<pos1<<", pos2: "<<pos2<<endl;
		system("pause");
	}
	return true;
}

//找到当前种群中适应度最好的个体，存入BestSolution；计算适应度的总和
//函数更新BestUnImproved，在每次迭代过程的最后调用一次，注意不可重复调用
void CGA::FindBestSolution()
{
	unsigned int i;
	bool improved = false;
	double best_fitness_value = 0;
	this->sumFitnessValue = 0;
	for (i=0; i < pSolution.size(); i++)
	{
		if(pSolution[i].getFitnessValue() > BestSolution.getFitnessValue())
		{
			BestSolution = pSolution[i];
			improved = true;
		}
	}
	if (improved)
	{
			BestUnImproved = 0;
			//BestSolution.PrintSolution();
	}
	else
		BestUnImproved++;
			
	best_fitness_value = BestSolution.getFitnessValue();
	for (i=0; i < pSolution.size(); i++)
	{
		//计算相对适应度的总和，用于轮盘赌选择
		this->sumFitnessValue += pSolution[i].getFitnessValue()/BestSolution.getFitnessValue()/10;
	}
}

//找到当前种群中适应度最差的个体用于替换
//将<var>worst_pos</var>指向它
void CGA::FindWorstSolution()
{
	unsigned int i;
	unsigned int worst1 = 0, worst2 = 0;
	for (i=0; i < pSolution.size(); i++)
	{
		if(pSolution[i].getFitnessValue() < pSolution[worst2].getFitnessValue())
		{
			if(pSolution[i].getFitnessValue() < pSolution[worst1].getFitnessValue())
			{
				worst2 = worst1;
				worst1 = i;
			}
			else
				worst2 = i;
		}
	}
	this->worst_first = worst1;
	this->worst_second = worst2;
}

//判断一个个体是否与当前种群中的个体相同
bool CGA::IfExists(CGA_Solution& it) 
{
	unsigned int i;
	for(i = 0; i < pSolution.size(); i++)
	{
		if(pSolution[i].SameAs(it))
			return true;
	}
	return false;
}

//判断是否符合结束条件，即达到MAX_UNIMPROVED_NUM和MAX_SOLUTION_NUM
bool CGA::OptCompleted()
{
	return ((NumberofSolution >= MAX_SOLUTION_NUM) && (NumberofUnImproved >= MAX_UNIMPROVED_NUM));
}

void CGA::finalstatus()
{
	cout<<"*************************************************"<<endl;
	cout<<"Number of Solutions is "<< NumberofSolution <<"."<<endl;
	cout<<"Number of Unimproved is "<< NumberofUnImproved <<"."<<endl;
	cout<<"*************************************************"<<endl;
	cout<<"sensor_num list:"<<endl;
	for(unsigned int i=0;i<pSolution.size();i++)
	{
		cout<<pSolution[i].sensor_num<<"  ";
		if(i % 10 == 9)
			cout<<endl;
	}
	cout<<"*************************************************"<<endl;
}
