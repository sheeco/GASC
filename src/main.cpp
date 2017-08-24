#include "GlobalParameters.h"
#include "Topology.h"
#include "Preprocess.h"
#include "Optimization.h"
#include "GA.h"

using namespace std;

//目标函数中的系数6
double RATE_SENSOR_NUM  = 1.0;
double RATE_DISTANCE  = 0;

int POPULATION_SIZE = 200;
int MAX_UNIMPROVED_NUM = 30;
int MAX_SOLUTION_NUM = 40000;
double PROB_CROSSOVER = 1.0;  //交叉概率
double PROB_MUTATION = 1.0;  //变异概率
	
ofstream savetofile;
ofstream parameters;

unsigned int BUFF_MAX = 840;
unsigned int M_VELOCITY = 3;
unsigned int NumberofTopofile = 12;
CTopology topo;

string pathTopology = "res/topo/";
string strsTopology[] = { "topology50","topology60","topology70","topology80","topology90","topology100","topology110","topology120","topology130","topology140",\
"topology150","topology200" };

int main(/*int args,char* argv[]*/) //输入两个参数，第一个参数为速度，第二个参数为buffer
{
	//M_VELOCITY = atoi(argv[1]);
	//BUFF_MAX = atoi(argv[2]);
	srand((unsigned)time(NULL));
	
	parameters = ofstream("parameters.txt", ios::out|ios::app);
	parameters<<"[with Ratio]"<<endl;
	parameters<<"[Parameters]"<<endl;
	parameters<<"POP_SIZE: "<<POPULATION_SIZE<<endl;
	parameters<<"MAX_SOLUTION: "<<MAX_SOLUTION_NUM<<endl;
	parameters<<"MAX_UNIMPROVED: "<<MAX_UNIMPROVED_NUM<<endl;
	parameters<<"PROB_CROSSOVER: "<<PROB_CROSSOVER<<endl;
	parameters<<"PROB_MUTATION: "<<PROB_MUTATION<<endl;

	//连续运行10遍
	//for(unsigned int j=0;j < 5;j++)
	for(unsigned int j = 0; j < 1; j++)
	{
	//for(unsigned int i=0;i < NumberofTopofile;i++)
	for(unsigned int i = 11;i > 10; i--)
	{
		savetofile = ofstream("sym_test_parameters.txt", ios::out|ios::app);
		Preprocess  *data=new Preprocess(pathTopology + strsTopology[i]);
		topo=data->GenerateCTopology();//topo赋值操作
		cout<<strsTopology[i]<<endl;
		parameters<<strsTopology[i]<<endl;
		savetofile<<strsTopology[i]<<endl;
		CGA *Opt=new CGA();
		Opt->GenerateInit();
		Opt->Optimize();
		cout<<"[Parameters]"<<endl;
		cout<<"topo: "<<(topo.listPOI.size() - 1)<<endl;
		cout<<"POP_SIZE: "<<POPULATION_SIZE<<endl;
		cout<<"MAX_SOLUTION: "<<MAX_SOLUTION_NUM<<endl;
		cout<<"MAX_UNIMPROVED: "<<MAX_UNIMPROVED_NUM<<endl;
		cout<<"PROB_CROSSOVER: "<<PROB_CROSSOVER<<endl;
		cout<<"PROB_MUTATION: "<<PROB_MUTATION<<endl;
		cout<<"NUM_ITERATION: "<<Opt->runninground<<endl;
		savetofile<<"*****************************************"<<endl;
		savetofile<<"[Parameters]"<<endl;
		savetofile<<"topo: "<<(topo.listPOI.size() - 1)<<endl;
		savetofile<<"POP_SIZE: "<<POPULATION_SIZE<<endl;
		savetofile<<"MAX_SOLUTION: "<<MAX_SOLUTION_NUM<<endl;
		savetofile<<"MAX_UNIMPROVED: "<<MAX_UNIMPROVED_NUM<<endl;
		savetofile<<"PROB_CROSSOVER: "<<PROB_CROSSOVER<<endl;
		savetofile<<"PROB_MUTATION: "<<PROB_MUTATION<<endl;
		savetofile<<"NUM_ITERATION: "<<Opt->runninground<<endl;

		cout<<"*****************************************"<<endl;
		savetofile<<"*****************************************"<<endl;
		cout<<"Best Solution:"<<endl;
		savetofile<<"Best Solution:"<<endl;
		Opt->BestSolution.PrintSolution();
		cout<<"*****************************************"<<endl;
		savetofile<<"*****************************************"<<endl;
		
		savetofile.close();
		topo.ClearDistanceMatrix();
		delete Opt;
		delete data;
	}
	}
		parameters.close();
	    //system("pause");
}