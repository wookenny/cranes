#include "Main.h"

#include <algorithm>
#include "Instance.h"

void read_instance(std::vector<std::string> argv){
	if (argv.size()!=1){
		std::cout<<"read [file] \n \tRead the input file and print the instance."<<std::endl;
		return;
	}
	
	//read file and give statistics
	Instance inst(argv[0]);
	std::cout<< inst <<std::endl;
}

void print_random_instance(std::vector<std::string> argv){
	if (argv.size()<2 || argv.size()>3){
		std::cout<<"random [k] [n] <s>\n \tGenerates a random instance with k vehicles, \n\tn jobs and with seed s. Default seed is 0."<<std::endl;
		return;
	}
	
	unsigned int seed = 0;
	Instance i;
	i.set_num_vehicles(std::stoi(argv[0]));
	if(argv.size()==3)
		seed = std::stoi(argv[2]);
	i.generate_random_depots(-100,100,-10,10,seed);
	i.generate_random_jobs(std::stoi(argv[1]),-100,100,-10,10,seed);
	
	std::cout<< i <<std::endl;
}


void test_mip(std::vector<std::string> argv){
	if (argv.size()>2){
		std::cout<<"test_mip <n> ,k>\n Runs some tests on the mip formulation!";
		std::cout<<"\n\tn: number of jobs(default = 4)\n\tk: number of vehicles(default = 2)"<<std::endl;
		return;
	}
	//set default parameter and parse given values
	int k = 2;
	int jobs = 4;
	if(argv.size()>0)
		jobs = std::stoi(argv[0]);
	if(argv.size()>1)
		k = std::stoi(argv[1]);
		
	Instance i(k);
	//i.generate_random_depots(-10, 10, -10, 10, 0);
	for(int j=0; j<k;++j)
		i.add_depotposition(std::array<int, 2>{{0,0}});

	i.generate_random_jobs(  jobs, -10, 10, -10, 10, 0);
	Tours &&t = i.get_MIP_solution();
	
	std::cout<<i<<std::endl;
	std::cout<<std::boolalpha;
	std::cout<<"MIP-Solution valid: "<<i.verify(t)<<std::endl;
	std::cout<<"Makespan: "<<i.makespan(t)<<std::endl;
	std::cout<<t<<std::endl;
}

