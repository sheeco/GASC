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

//��ʼ���������main��������
void CGA::GenerateInit(void)
{
	unsigned int i;
	unsigned int temp=65535;
//DBG:
	cout<<"��ʼ��������..."<<endl;

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
	cout<<"��ʼ��������ɣ�"<<endl;

	FindBestSolution();
	FindWorstSolution();
}

//�Ż�������������Ҫ���̣���main��������
CSolution CGA::Optimize(void)
{
	double bet1 = 0;
	int bet2 = 0;
	bool performCrossover = false;  //�Ƿ�ʵʩ����
	bool performMutation = false;  //�Ƿ�ʵʩ����
	bool improved = false;  //�Ƿ������Ⱥ
	CGA_Solution *output1, *output2, *output3, *output4;  //���ڱ��潻�桢�����Ľ��

	while(!OptCompleted())//�Ż�δ�ﵽ�ٽ�����
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

		//���̶�ѡ����������
		SelectTwo();
//DBG:
		//printf("Selected indivs: %d, %d\n", current_op1, current_op2);
		//pSolution[current_op1].PrintSolution();
		//pSolution[current_op2].PrintSolution();

		bet1 = RandomFloat(0, 1);

//DBG:
		//cout<<"bet1: "<<bet1<<endl;

		//���������ʵʩ����
		if(bet1 <= PROB_CROSSOVER)
		{
			//���ʵʩ���ֽ��������е�һ��
			bet2 = RandomInt(0, 3);
//DBG:
			//cout<<"bet2 : "<<bet2<<endl;

			switch(bet2)
			{
			case 0:
				//cout<<"�Գ�ƥ�佻��"<<endl;
				SymmCrossover(pSolution[current_op1], pSolution[current_op2], *output1, *output2);
				break;
			case 1:
				//cout<<"�ǶԳ�ƥ�佻��"<<endl;
				UnSymmCrossover(pSolution[current_op1], pSolution[current_op2], *output1, *output2);
				break;
			case 2:
				//cout<<"��·����"<<endl;
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
		//�����ʵʩ���棬ֱ�Ӹ��Ʊ�ѡ�е��������壬����ʵʩ����
		else
		{
//DBG:
			cout<<"��ʵʩ����        ";

			*output1 = pSolution[current_op1];
			*output2 = pSolution[current_op2];
		}

		//���������ʵʩ����
		bet1 = RandomFloat(0, 1);
//DBG:
		//cout<<"bet1: "<<bet1<<endl;

		if(bet1 <= PROB_MUTATION)
		{
			//���ʵʩ���ֱ��������е�һ��
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
		//�����ʵʩ���죬ֱ�Ӹ���output1, output2��Ϊ���ε����Ľ��
		else
		{
//DBG:
			cout<<"		��ʵʩ����\n";

			*output3 = *output1;
			*output4 = *output2;
		}

		/*���output3, output4��ԭ��Ⱥ��Ⱦɫ�岻��ͬ��
		  ����Ӧ��ֵ���ڻ����ԭ��Ⱥ�����ĸ��壬����и���*/
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
	//�Ż���������ӡ���ս��
	finalstatus();
	return BestSolution;
}

/*���ֽ��桢��������*/
//�Գ�ƥ�佻��
bool CGA::SymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2)
{
	output1 = indiv1;
	output2 = indiv2;

	//FIXME:���Ҫ���ж�����·�ĶԳƽ��棬����crossover_num���ɣ����ѡ�����·�ſ����ظ�����
	//���н����·�ߵ�����
	int crossover_num = 1;
	int i, j;
	int selected_route_index1, selected_route_index2;
	LISTROUTE::iterator curr_route1, curr_route2;
	
	for(j = 0; j < crossover_num; j++)
	{
//RATIO:
		//����Ratioѡ������·��
		output1.SelectRouteByRatio(selected_route_index1);
		output2.SelectRouteByRatio(selected_route_index2);
		//���ѡ��һ��·��
		//selected_route_index1 = RandomInt(0, output1.routelist.size());
		//selected_route_index2 = RandomInt(0, output2.routelist.size());
		//ɾ����ѡ�е�����·������������
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
		//��ɾ����Ҫ�����·�����Ѵ��ڵ�POI
		output1.DeletePOIList(selected_route2.seq);
		output2.DeletePOIList(selected_route1.seq);
		//������·
		output1.routelist.insert(curr_route1, selected_route2);
		output2.routelist.insert(curr_route2, selected_route1);
		//�ռ�����ȱ�ٵ�POI�������ٲ���
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

	cout<<"�Գ�ƥ�佻�����";
	return true;
}

//�ǶԳ�ƥ�佻��
bool CGA::UnSymmCrossover(CGA_Solution &indiv1, CGA_Solution &indiv2, CGA_Solution &output1, CGA_Solution &output2)
{
	output1 = indiv1;
	output2 = indiv2;

	//FIXME:���Ҫ��ǶԳƽ������·�����������������crossover_num1, crossover_num1�ĳ�ʼ��
	//���н����·�ߵ���������output1��ѡ��num1����·����output2��ѡ��num2����·
	int crossover_num1, crossover_num2;
	crossover_num1 = 1;
	crossover_num2 = 2;
	int i, j, k;
	//FIXME:���Ҫ��ǶԳƽ������·��������������˴�����Ҫ����
	int selected_route_index1[1];
	int selected_route_index2[2];
	LISTROUTE::iterator curr_route1, curr_route2;
	BASIC_ROUTE_Struct selected_route1[1];
	BASIC_ROUTE_Struct selected_route2[2];
	
	//���ѡ��������·��
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

	//��ɾ���±��С��·��
	if(selected_route_index2[0] < selected_route_index2[1])
	{
		int tmp;
		tmp = selected_route_index2[0];
		selected_route_index2[0] = selected_route_index2[1];
		selected_route_index2[1] = tmp;
	}

	//ɾ����ѡ�е�·������������
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

	//��ɾ����Ҫ�����·�����Ѵ��ڵ�POI
	for(k = 0; k < crossover_num2; k++)
		output1.DeletePOIList(selected_route2[k].seq);
	for(k = 0; k < crossover_num1; k++)
		output2.DeletePOIList(selected_route1[k].seq);
	//������·
	for(k = 0; k < crossover_num2; k++)
		output1.routelist.insert(curr_route1, selected_route2[k]);
	for(k = 0; k < crossover_num1; k++)
		output2.routelist.insert(curr_route2, selected_route1[k]);
	//�ռ�����ȱ�ٵ�POI�������ٲ���
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

	cout<<"�ǶԳ�ƥ�佻�����";
	return true;
}

//��·����
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
	//���ѡ��һ��·��
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

	//ȥ�أ������ٲ���
	if(!output1.ReInsertPOIList(selected_route2.seq))
		return false;
	if(!output2.ReInsertPOIList(selected_route1.seq))
		return false;

	//else 
	{
		cout<<"��·�������      ";
		return true;
	}
}

//��·����(R1����)
bool CGA::RouteMutution(CGA_Solution &indiv1, CGA_Solution &output1)
{
	output1 = indiv1;

	int i;
	int selected_route_index;
	LISTROUTE::iterator curr_route;
//RATIO:
	output1.SelectRouteByRatio(selected_route_index);
	//���ѡ��һ��·��
	//selected_route_index = RandomInt(0, output1.routelist.size());
	//ɾ����ѡ�е�·��
	for(i = 0, curr_route = output1.routelist.begin(); i < selected_route_index && curr_route != output1.routelist.end(); i++)
	{
		curr_route++;
	}
	BASIC_ROUTE_Struct selected_route;
	selected_route = *curr_route;
	output1.routelist.erase(curr_route);
	output1.UpdateStatus();
	//��ɾ����POI���ٲ��뵽ԭ·����ȥ
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
		cout<<"		��·�������";
		return true;
	}
}

//�������(R2����)
bool CGA::POIMutution(CGA_Solution &indiv1, CGA_Solution &output1)
{
	output1 = indiv1;

	int i;
	int selected_poi_seq;
	int selected_route_index;
	LISTROUTE::iterator curr_route;
	//���ѡ��һ��POI
	selected_poi_seq = RandomInt(1, topo.itotalPOINumber);
	if(output1.IfPOIExists(selected_poi_seq, selected_route_index))
	{
		for(i = 0, curr_route = output1.routelist.begin(); i < selected_route_index && curr_route != output1.routelist.end(); i++)
			curr_route++;
		CRoute new_route(*curr_route);
		//��ѡ�е�POI���²��뵽ԭ·������
		new_route.DeleteFromRoute(selected_poi_seq);
		new_route.SolomenInsert(selected_poi_seq);
		output1 = output1;
		for(i = 0, curr_route = output1.routelist.begin(); i < selected_route_index && curr_route != output1.routelist.end(); i++)
			curr_route++;
		*curr_route = new_route.current_route;
		output1.UpdateStatus();
		cout<<"		����������";
		return true;
	}
	else
		return false;
}

/*Supportive Functions*/

//�����̶�ѡ���������壬���� <var>current_op1</var> �� <var>current_op2</var> ָ������
//����ɹ�������true
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

//�ҵ���ǰ��Ⱥ����Ӧ����õĸ��壬����BestSolution��������Ӧ�ȵ��ܺ�
//��������BestUnImproved����ÿ�ε������̵�������һ�Σ�ע�ⲻ���ظ�����
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
		//���������Ӧ�ȵ��ܺͣ��������̶�ѡ��
		this->sumFitnessValue += pSolution[i].getFitnessValue()/BestSolution.getFitnessValue()/10;
	}
}

//�ҵ���ǰ��Ⱥ����Ӧ�����ĸ��������滻
//��<var>worst_pos</var>ָ����
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

//�ж�һ�������Ƿ��뵱ǰ��Ⱥ�еĸ�����ͬ
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

//�ж��Ƿ���Ͻ������������ﵽMAX_UNIMPROVED_NUM��MAX_SOLUTION_NUM
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
