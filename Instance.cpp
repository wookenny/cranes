#include "Instance.h"

#include <boost/regex.hpp>
#include <fstream>
#include <random>

#include "MIP.h"
#include "Job.h"

const boost::regex empty("\\s*");
const boost::regex comment("\\s*//.*"); //line starts with // after whitespace
const boost::regex vehicles("\\s*NUM_CRANES\\s*=\\s*(\\d+)"); //number of vehicles, captures it
const boost::regex depot_start("\\s*START_DEPOTS\\s*=\\s*(.*)");//fist match this
const boost::regex successive_depot("\\s*\\(([^;]*);([^\\)]+)\\)[^\\(]*(.*)");//extract each depot as single call
const boost::regex forward_job("\\s*\\(([^;]+);([^\\)]+)\\)\\s*->\\s*\\(([^;]+);([^\\)]+)\\)\\s*"); 
const boost::regex backward_job("\\s*\\(([^;]+);([^\\)]+)\\)\\s*<-\\s*\\(([^;]+);([^\\)]+)\\)\\s*");

using namespace std;	

string Instance::to_string() const{
	string inst_str = "2D-VS Instance with "+std::to_string(_num_vehicles)+" vehicles.\n";
	inst_str += "Depots: ";
	for(auto &d: _depotPositions)
		inst_str += "("+std::to_string(d[0])+"; "+std::to_string(d[1])+") ";
	inst_str += "\nJobs:\n";
	for(auto &j: _jobs)
		inst_str += j.to_string()+"\n";

	return inst_str;
}
		
		
Instance::Instance(string file){
	_num_vehicles = 0;
	//open file an read line after line
	fstream infile;
	infile.open(file.c_str());
	string line;
	//warn if file is bad!
	if(!infile){
		cerr<<"Warning: Could not load the file "<<file<<"! Creating empty instance."<<endl;
		return;
	}	
	
	//read every single line and find the matching regex
	while( getline(infile,line) ){
		_parse_line(line);	
	}
	
	infile.close();
	
	if(_num_vehicles!=_depotPositions.size())
		cerr<< "Warning: Number of vehicles and given depot positions does not fit!" <<endl;
}


void Instance::_parse_line(string &line){
	if(boost::regex_match (line,comment) || boost::regex_match (line,empty)){
		return; //nothing to do on a comment
	}
		
	boost::cmatch cm;
	//how many vehicles?
	if( boost::regex_match (line.c_str(),cm,vehicles)){
		//already an entry stored?
		if( _num_vehicles >0)
			cerr<<"Warning: Multiple lines for number of vehicles!"<<endl;
			
		 _num_vehicles = stoi(cm[1]);
		 return;
	}
	
	//depot positions?
	if(boost::regex_match (line.c_str(),cm,depot_start)){
		
		line = cm[1];
		while(boost::regex_match (line.c_str(),cm,successive_depot)){
			_depotPositions.push_back(array<int, 2>{{stoi(cm[1]),stoi(cm[2])}});
			line = cm[3];
		}
		return;
	}
	
	
	//forward job?
	if( boost::regex_match (line.c_str(),cm,forward_job)){
		Job j(_jobs.size()+1,stoi(cm[1]),stoi(cm[2]),
						   stoi(cm[3]),stoi(cm[4]));
		_jobs.push_back( j );
		return;
	}
	
	//backward job?
	if( boost::regex_match (line.c_str(),cm,backward_job)){
		Job j(_jobs.size()+1,stoi(cm[3]),stoi(cm[4]),
						   stoi(cm[1]),stoi(cm[2]));
		_jobs.push_back( j );
		return;
	}
	//if nothing was matching, return false				
	cerr<<"Warning: Could not parse this line: \""<<line<<"\""<<endl;

}

void Instance::generate_random_jobs(int n, int min_x, int max_x, int min_y, int max_y, unsigned int seed){
	
	//random number generation via mersenne twister
	mt19937_64 rng;
	rng.seed(seed);
	//unid distribution
	uniform_int_distribution<int> dist_x(min_x,max_x); 
	uniform_int_distribution<int> dist_y(min_y,max_y);
	
	while(0<=--n){
		Job j(_jobs.size()+1,dist_x(rng),dist_y(rng),dist_x(rng),dist_y(rng));
		_jobs.push_back( j );
	}
}

void Instance::generate_random_depots(int min_x, int max_x, 
										int min_y, int max_y, unsigned int seed){
	//random number generation via mersenne twister
	mt19937 rng;
	rng.seed(seed);
	//unid distribution
	uniform_int_distribution<int> dist_x(min_x,max_x); 
	uniform_int_distribution<int> dist_y(min_y,max_y); 
	//creates depots until the number the nu,ber of vehicles matches
	while( _depotPositions.size() < _num_vehicles)
		_depotPositions.push_back( array<int,2>{{dist_x(rng),dist_y(rng)}});
}
		
//TODO: add GTests for all cases
bool Instance::verify(Tours& t) const{
	double EPS = 0.5;
	t.sort_jobs();

	//Tests: 
	//1. every job included?
	for(auto j = _jobs.begin(); j!=_jobs.end(); ++j )
		if(!t.contains(&*j)){
			cout<<"Job "<<*j<<" missing!"<<endl;
			return false;
		}
	//2. no other job included?
	//easy check, because no job can be in the tour twice and therefore we only
	//need to check the number
	if(_jobs.size()!=t.num_jobs()){
		cout<<"Wrong number of jobs. Needed "<<_jobs.size()
					<<" found "<<t.num_jobs()<<endl;
		return false;
	}
		
	//3. every tour consistent in itself?
	for(uint i=0; i<t.num_tours(); ++i){
		if(t[i].size()==0)
			continue;
		//first job reachable from depot?
		double time1; const Job* job1;

		tie(job1, time1) = t[i].front();
		if( dist_inf(job1->get_alpha(),_depotPositions[i]) > time1+EPS  ){
			cout<< "Job "<<*job1<<" not reachable from depot"<<endl;
			cout<<"starting times: "<<time1<<endl;
			cout<< "dist: "<<dist_inf(job1->get_alpha(),_depotPositions[i])<<endl;
			cout<<"difference: "<<time1-dist_inf(job1->get_alpha(),_depotPositions[i])<<endl;
			return false;
		}
			
		for(uint j=1; j<t[i].size(); ++j){
			double time2; const Job* job2;
			tie(job1, time1) = t[i][j-1];
			tie(job2, time2) = t[i][j];		
			if(dist_inf(job1->get_beta(),job2->get_alpha())+job1->length() > (time2 - time1 )+EPS){
				cout<< "Job "<<*job2<<" not reachable after job "<<*job1<<endl;
				cout<<"starting times: "<<time1 << " and "<<time2<<endl;
				cout<<time1<< " + "<<job1->length()<<" + "<<dist_inf(job1->get_beta(),job2->get_alpha()) <<" - EPS "
				<< " = " << (time1+job1->length()+dist_inf(job1->get_beta(),job2->get_alpha())) << " - EPS > "<<time2<<endl;
				return false;
			}	
		}
		
	}
	
	//4. all tours collison-free
	//TODO How To Check This?? solving the 1DVS?  
	
	
	//no error found => return true!
	return true;
}


/*
Calculates the makespan for a given solution.
*/
double Instance::makespan(Tours& t) const{
	//tours should be valid!
	//assert(verify(t));
	t.sort_jobs();
	
	//find latest return of a job
	double makespan = 0;
	for(uint i=0; i < t.num_tours(); ++i){
	    if(0==t[i].size())
	    	continue;
		const Job* j = get<0>(t[i].back());
	 	double time = get<1>(t[i].back()) + j->length();
	 	time += dist_inf(_depotPositions[i], j->get_beta());
		makespan = max(makespan, time );
	}
	
	return makespan;
}

Tours Instance::get_MIP_solution(bool collision_free,bool LP_relax,bool debug) const{
	
	MIP mip(*this);
	//Hint: Methods returns a Tours objct.
	//To get rid off copying overhead, the rvalue reference
	//grants perfect forwarding.
	Tours&& t = mip.solve(collision_free, LP_relax,debug);
	//check MIP
	//TODO: uncomment this
	//assert(verify(t));
	return t;
}

//TODO: use christofides to find a better single TSP solution
//calculates an upper bound for the optimal tour
//right now we just tae all jobs in the given order and calculate a tour from depot 0
unsigned int Instance::get_upper_bound() const{
	unsigned int length = 0;
	//save current position and add length to next job and its length
	auto current_position = _depotPositions[0];
	for(const Job& j: _jobs){
		length += dist_inf<int>(current_position,j.get_alpha());
		length += j.length();
		current_position = j.get_beta();
	}
	//add last transition to the depot
	length += dist_inf<int>(current_position,_depotPositions[0]);
	return length;
}




		

