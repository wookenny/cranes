#include "Main.h"

#include <algorithm>
#include <unordered_map>
#include <memory>

#include "generalizedVRP_MIP.h"
#include "m_TSP_MIP.h"
#include "Instance.h"

using namespace std;

void read_instance(vector<string> argv){
	if (argv.size()!=1){
		cout<<"read [file] \n \tRead the input file and print the instance."<<endl;
		return;
	}
	
	//read file and give statistics
	Instance inst(argv[0]);
	cout<< inst <<endl;
}

void print_random_instance(vector<string> argv){
	if (argv.size()<2 || argv.size()>3){
		cout<<"random [k] [n] <s>\n \tGenerates a random instance with k vehicles, \n\tn jobs and with seed s. Default seed is 0."<<endl;
		return;
	}
	
	unsigned int seed = 0;
	Instance i;
	i.set_num_vehicles(stoi(argv[0]));
	if(argv.size()==3)
		seed = stoi(argv[2]);
	i.generate_random_depots(-100,100,-10,10,seed);
	i.generate_random_jobs(stoi(argv[1]),-100,100,-10,10,seed);
	
	cout<< i <<endl;
}

void test_mtsp_mip(vector<string> argv){
	if (argv.size()>3 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"test_mip <n> <k> <seed>\n Runs some tests on the mip formulation!";
		return;
	}
	//set default parameter and parse given values
	int k = 2;
	int jobs = 20;
	int seed = 0;
	
	if(argv.size()>0)
		jobs = stoi(argv[0]);
	if(argv.size()>1)
		k = stoi(argv[1]);
	if(argv.size()>2)
		seed = stoi(argv[2]);

	Instance i(k);
	//i.generate_random_depots(-10, 10, -10, 10, 0);
	for(int j=0; j<k;++j)
		i.add_depotposition(array<int, 2>{{0,0}});

	i.generate_random_jobs(  jobs, -10, 10, -10, 10, seed);
	unique_ptr<generalizedVRP_MIP> mip_ptr(new m_TSP_MIP(i));
	mip_ptr->set_debug(true);
	Tours &&t = mip_ptr->solve();

	
	cout<<i<<endl;
	cout<<boolalpha;
	cout<<"MIP-Solution valid: "<<i.verify(t)<<endl;
	cout<<"Makespan: "<<i.makespan(t)<<endl;
	cout<<t<<endl;
}




void test_mip(vector<string> argv){
	if (argv.size()>5 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"test_mip <n> <k> <coll.> <LP> <seed>\n Runs some tests on the mip formulation!";
		cout<<"\n\tn: number of jobs(default = 4)\n\tk: number of vehicles(default = 2)"<<endl;
		cout<<"\tcoll.: collision-avoidance constraints adding (default = true)"<<endl;
		cout<<"\tLP: just solve the LP relaxation(default = false)"<<endl;
		cout<<"\tseed: seed used to genenerate the jobs.(default = time(0))"<<endl;
		return;
	}
	//set default parameter and parse given values
	int k = 2;
	int jobs = 4;
	bool collision = true,LP = false;
	int seed = time(0);
	
	//parse the given ones
	if(argv.size()>0)
		jobs = stoi(argv[0]);
	if(argv.size()>1)
		k = stoi(argv[1]);
	unordered_map<string,bool> string_to_bool =  {{"t",true},{"true",true},
			{"1",true},{"yes",true},{"f",false},{"false",false},
			{"0",false},{"no",false},{"y",true},{"n",false} };

	if(argv.size()>2)
		if(string_to_bool.find(argv[2])!=string_to_bool.end())
			collision = string_to_bool[argv[2]];
	if(argv.size()>3)
		if(string_to_bool.find(argv[3])!=string_to_bool.end())
			LP = string_to_bool[argv[3]];		
	if(argv.size()>4)
		seed = stoi(argv[4]);
	//give some informations
	cout<<boolalpha;
	cout<< "Solving a 2D-VS instance with the following settings:\n";
	cout<< "Jobs: "<<jobs<<"\nVehicles: "<<k;
	cout<<"\nAdding collision avoidance constr.: "<<collision;
	cout<<"\nSolving LP relaxation.: "<<LP<<endl;
		
	Instance i(k);
	//i.generate_random_depots(-10, 10, -10, 10, 0);
	for(int j=0; j<k;++j)
		i.add_depotposition(array<int, 2>{{0,0}});

	i.generate_random_jobs(  jobs, -10, 10, -10, 10, seed);
	Tours &&t = i.get_MIP_solution(collision, LP);

	
	cout<<i<<endl;
	cout<<boolalpha;
	if(not LP) cout<<"MIP-Solution valid: "<<i.verify(t)<<endl;
	cout<<"Makespan: "<<i.makespan(t)<<endl;
	cout<<t<<endl;
}

//TODO: REMOVE THIS AFTER IMPLEMENTING A CHRISTOFIDES-LIKE Heuristic 
void test_PM(std::vector<std::string> argv){
	if (argv.size()<1 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"match [n] \n Calculates a min.";
		cout<<"\tcost perfect matching on a random graph of size n."<<endl;
		return;
	}
	int n = stoi(argv[0]);
	run_PM(n);
}


