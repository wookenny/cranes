#include "Instance.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp> //used for the time things
#include <boost/date_time/gregorian/gregorian.hpp>   //used for easier date manipulation
#include <fstream>
#include <random>
#include <chrono>
#include <ctime>

#include "Job.h"
#include "generalizedVRP_MIP.h"
#include "m_TSP_MIP.h"
#include "independent_TSP_MIP.h"

const boost::regex empty("\\s*");
const boost::regex comment("\\s*//.*"); //line starts with // after whitespace
const boost::regex vehicles("\\s*NUM_CRANES\\s*=\\s*(\\d+)"); //number of vehicles, captures it
const boost::regex safety_dist("\\s*SAFETY_DIST\\s*=\\s*(\\d+)"); //safety dist, captures it
const boost::regex depot_start("\\s*START_DEPOTS\\s*=\\s*(.*)");//fist match this
const boost::regex successive_depot("\\s*\\(([^;]*);([^\\)]+)\\)[^\\(]*(.*)");//extract each depot as single call
//additional number in front of job as non-capturing group ->
// "(?:\\[[0-9]+\\]:){0,1}\\s*"
const boost::regex forward_job("(?:\\[[0-9]+\\]:){0,1}\\s*\\(([^;]+);([^\\)]+)\\)\\s*->\\s*\\(([^;]+);([^\\)]+)\\)\\s*"); 
const boost::regex backward_job("(?:\\[[0-9]+\\]:){0,1}\\s*\\(([^;]+);([^\\)]+)\\)\\s*<-\\s*\\(([^;]+);([^\\)]+)\\)\\s*");

using namespace std;	

long Instance::num_checks = 0;

string Instance::to_string() const{
	string inst_str = "2D-VS Instance with "+std::to_string(num_vehicles_)+" vehicles.\n";
	if(safety_distance_>0)
	    inst_str += "Safety Distance: "+std::to_string(safety_distance_)+"\n"; 
	inst_str += "Depots: ";
	for(auto &d: depotPositions_)
		inst_str += "("+std::to_string(d[0])+"; "+std::to_string(d[1])+") ";
	inst_str += "\nJobs:\n";
	for(auto &j: jobs_)
		inst_str += j.to_string()+"\n";

	return inst_str;
}
		
		
Instance::Instance(string file){
	num_vehicles_ = 0;
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
		parse_line_(line);	
	}
	
	infile.close();
	sort_depots_();
	
	if(num_vehicles_!=depotPositions_.size())
		cerr<< "Warning: Number of vehicles and given depot positions does not fit!" <<endl;
}


void Instance::parse_line_(string &line){
	if(boost::regex_match(line,comment) || boost::regex_match(line,empty)){
		return; //nothing to do on a comment
	}
		
	boost::cmatch cm;
	//how many vehicles?
	if( boost::regex_match (line.c_str(),cm,vehicles)){
		//already an entry stored?
		if( num_vehicles_ >0)
			cerr<<"Warning: Multiple lines for number of vehicles!"<<endl;
			
		 num_vehicles_ = stoi(cm[1]);
		 return;
	}
	
	//safety distance?
  if( boost::regex_match (line.c_str(),cm,safety_dist)){
		//already an entry stored?
		if( safety_distance_ >0)
			cerr<<"Warning: Multiple lines for safety distance!"<<endl;
		 safety_distance_ = stoi(cm[1]);
		 return;
	}
	
	//depot positions?
	if(boost::regex_match (line.c_str(),cm,depot_start)){
		
		line = cm[1];
		while(boost::regex_match (line.c_str(),cm,successive_depot)){
			depotPositions_.push_back(array<int, 2>{{stoi(cm[1]),stoi(cm[2])}});
			line = cm[3];
		}
		return;
	}
	
	
	//forward job?
	if( boost::regex_match (line.c_str(),cm,forward_job)){
		Job j(jobs_.size()+1,stoi(cm[1]),stoi(cm[2]),
						   stoi(cm[3]),stoi(cm[4]));
		jobs_.push_back( j );
		return;
	}
	
	//backward job?
	if( boost::regex_match (line.c_str(),cm,backward_job)){
		Job j(jobs_.size()+1,stoi(cm[3]),stoi(cm[4]),
						   stoi(cm[1]),stoi(cm[2]));
		jobs_.push_back( j );
		return;
	}
	//if nothing was matching, return false				
	cerr<<"Warning: Could not parse this line: \""<<line<<"\""<<endl;

}

void Instance::generate_random_jobs(int n, int min_x, int max_x,
                                    int min_y, int max_y, unsigned int seed){
	
	//random number generation via mersenne twister
	mt19937_64 rng;
	rng.seed(seed);
	//unid distribution
	uniform_int_distribution<int> dist_x(min_x,max_x); 
	uniform_int_distribution<int> dist_y(min_y,max_y);
	
	while(0<=--n){
		Job j(jobs_.size()+1,dist_x(rng),dist_y(rng),dist_x(rng),dist_y(rng));
		jobs_.push_back( j );
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
	while( depotPositions_.size() < num_vehicles_)
		depotPositions_.push_back( array<int,2>{{dist_x(rng),dist_y(rng)}});
	
	//shift depots
	if(safety_distance_>0){
	    for(uint i=0; i<depotPositions_.size()-1;++i)
	        if( depotPositions_[i][0]+static_cast<int>(safety_distance_) >depotPositions_[i+1][0])
	            depotPositions_[i+1][0]= depotPositions_[i][0]+safety_distance_;
	}	
}
		
//TODO: add GTests for all cases
bool Instance::verify(Tours& t) const{
    ++num_checks;
	double EPS = 0.5;
	t.sort_jobs();

	//Tests: 

	//0. exact right number of vehicles?
	if(t.num_tours() != num_vehicles()){
		if(debug_)
			cout<<"Wrong number of tours: "
			<<t.num_tours()<<" tours and "<<num_vehicles()<<" vehicles"<< endl;
		return false;
	}

	//1. every job included?
	for(auto j = jobs_.begin(); j!=jobs_.end(); ++j )
		if(!t.contains(&*j)){
			if(debug_) cout<<"Job "<<*j<<" missing!"<<endl;
			return false;
		}

	//2. no other job included?
	//easy check, because no job can be in the tour twice and therefore we only
	//need to check the number
	if(jobs_.size()!=t.num_jobs()){
		if(debug_) cout<<"Wrong number of jobs. Needed "<<jobs_.size()
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
		if( dist_inf(job1->get_alpha(),depotPositions_[i]) > time1+EPS  ){
			if(debug_){
				cout<< "Job "<<*job1<<" not reachable from depot"<<endl;
				cout<<"starting times: "<<time1<<endl;
				cout<< "dist: "<<dist_inf(job1->get_alpha(),depotPositions_[i])<<endl;
				cout<<"difference: "<<time1-dist_inf(job1->get_alpha(),depotPositions_[i])<<endl;
			}
			return false;
		}
			
		for(uint j=1; j<t[i].size(); ++j){
			double time2; const Job* job2;
			tie(job1, time1) = t[i][j-1];
			tie(job2, time2) = t[i][j];		
			if(dist_inf(job1->get_beta(),job2->get_alpha())+job1->length() >
			                                             (time2 - time1 )+EPS){
				if(debug_){ 
					cout<< "Job "<<*job2<<" not reachable after job "
					    <<*job1<<endl;
					cout<<"starting times: "<<time1 << " and "<<time2<<endl;
					cout<<time1<< " + "<<job1->length()<<" + "
					  <<dist_inf(job1->get_beta(),job2->get_alpha()) <<" - EPS "
					  << " = " << (time1+job1->length()+
					       dist_inf(job1->get_beta(),job2->get_alpha())) 
					  << " - EPS > "<<time2<<endl;
				}
				return false;
			}	
		}
		
	}

	//4. all tours collision-free
	//check the direction induced by the four points of two jobs
	// valid if: not contradicting, ok with assigned vehicles
	//check pair of jobs on differnt vehicles	
	for(uint v1=0; v1<t.num_tours()-1; ++v1){
		for(uint v2=v1+1; v2<t.num_tours(); ++v2){
			for(uint i=0; i<t.num_jobs(v1); ++i){
				for(uint j=0; j<t.num_jobs(v2); ++j){
					//i left of j => 1 or -2 are bad outcomes
					
					//safety dist>0 => shift second job to the left,
					//shifting distance: vehicle difference * safety dist
					int ord = -2;
					if(safety_distance_ <= 0){
					    ord = Job::getOrdering(t[v1][i],t[v2][j]);
				    }else{
				        Job job_shifted( std::get<0>(t[v2][j]) );
				        auto time = std::get<1>(t[v2][j]);
				        job_shifted.shift(-1*safety_distance_*(v2-v1),0);
				        std::tuple<const Job*, double> second_job;
				        second_job = make_tuple(&job_shifted, time); 
				        ord = Job::getOrdering(t[v1][i],second_job); 
				    }				
					if(-2==ord or 1==ord){
						if(debug_){
							cout<< "Job "<<t[v1][i]<<" crosses with job "
							             <<t[v2][j]<<endl;
						}
						return false;
					}	
				}
			}
		}
	}

	//job with same vehicle: no direction enforced at all
	for(uint v=0; v<t.num_tours(); ++v){
		for(uint i=0; i+1< t.num_jobs(v); ++i){
			for(uint j=i+1; j<t.num_jobs(v); ++j){
				//job have to be reachable(0), every other thing is BAD!
				if( Job::getOrdering(t[v][i],t[v][j]) != 0){
					if(debug_){ 
						cout<<"error on tour "<<v+1 <<endl;
						cout<<"job "<<i+1<<" and "<<j+1<<" not compatible"<<endl;
						cout<< t[v][i]<<"\n"<<t[v][j] <<endl;
						cout<<"status: "<< Job::getOrdering(t[v][i],t[v][j])<<endl;
					}
					return false;
				}	
			}
	 	}	
	 }
	
	//no error found => return true!
	return true;
}


/*
Calculates the makespan for a given solution.
*/
double Instance::makespan(Tours& t) const{
	//tours should be valid!
	assert(verify(t));
	//t.sort_jobs();
	
	//find latest return of a job
	double makespan = 0;
	for(uint i=0; i < t.num_tours(); ++i){
	    if(0==t[i].size())
	    	continue;
		const Job* j = get<0>(t[i].back());
	 	double time = get<1>(t[i].back()) + j->length();
	 	time += dist_inf(depotPositions_[i], j->get_beta());
		makespan = max(makespan, time );
	}
	
	return makespan;
}

Tours Instance::get_MIP_solution(bool collision_free,bool LP_relax,bool debug) const{

    //TODO: somehow switch on the desired MIP
	auto mip = unique_ptr<generalizedVRP_MIP>(new independent_TSP_MIP(*this));
	//auto mip = unique_ptr<generalizedVRP_MIP>(new m_TSP_MIP(*this));
	//Hint: Methods returns a Tours objct.
	//To get rid off copying overhead, the rvalue reference
	//grants perfect forwarding.

    mip->set_debug(debug);
	mip->set_collision(collision_free);
	mip->set_LP(LP_relax);
	
	Tours&& t = mip->solve();
	assert(verify(t));
	return t;
}

//TODO: use christofides to find a better single TSP solution
//calculates an upper bound for the optimal tour
//right now we just tae all jobs in the given order and calculate a tour from depot 0
unsigned int Instance::get_upper_bound() const{
	unsigned int length = 0;
	//save current position and add length to next job and its length
	auto current_position = depotPositions_[0];
	for(const Job& j: jobs_){
		length += dist_inf<int>(current_position,j.get_alpha());
		length += j.length();
		current_position = j.get_beta();
	}
	//add last transition to the depot
	length += dist_inf<int>(current_position,depotPositions_[0]);
	return length;
}

void setNiceDayFormat(){
	using namespace boost::posix_time;
	time_facet* facet(new time_facet("%R, %d.%m.%Y"));
	std::cout.imbue(std::locale(std::cout.getloc(), facet));
}

void Instance::writeToFile(std::string filename, std::string comments) const{
	//open file
	std::ofstream file;
	file.open(filename.c_str());
	if( !file ) {
		std::cerr << "Error writing instance to "<<filename<<" ." << std::endl;
		return;	
	}	
	//write instance
	using namespace boost::posix_time;
	using namespace boost::gregorian;
	setNiceDayFormat();
	
	file<<"//this file was written automaticaly\n//date: "
		<< second_clock::local_time() << endl;
	//write comments
	if(comments!=""){
		std::vector<std::string> lines;
		boost::split_regex( lines, comments, boost::regex( "\n" ) );
		for(const auto &line: lines){
			file<<"//"<<line<<"\n";
		}
		file<<std::endl;
	}

	int cranes = num_vehicles();

	file<< "NUM_CRANES\t=\t "<< cranes<<std::endl;
	if(safety_distance_>0)
	    file<<"SAFETY_DIST\t=\t"<<safety_distance_<<"\n";
	file<< "START_DEPOTS\t=\t" ;
	for(auto &d: depotPositions_)
		file<<"("+std::to_string(d[0])+"; "+std::to_string(d[1])+") ";
	//write all jobs
	file<< "\n//Jobs:"<<std::endl;
	//write the jobs ordered in a nnice direction
	for(const auto &job: jobs_)
				file<< job <<endl;

	file<<"\n";
	file.close();
}

array<int,4> Instance::get_bounding_box() const{
	array<int,4> bbox;
	bbox[0] = bbox[2] = depotPositions_[0][0];//x coords
	bbox[1] = bbox[3] = depotPositions_[0][1];//y coords
	
	//for all jobs, for all depots, save min/max coordinates
	for(auto &job: *this){
		bbox[0] = min(bbox[0],job.alpha()[0]); bbox[0] = min(bbox[0],job.beta()[0]);
		bbox[1] = min(bbox[1],job.alpha()[1]); bbox[1] = min(bbox[1],job.beta()[1]);
		bbox[2] = max(bbox[2],job.alpha()[0]); bbox[2] = max(bbox[2],job.beta()[0]);
		bbox[3] = max(bbox[3],job.alpha()[1]); bbox[3] = max(bbox[3],job.beta()[1]);
	}
	
	//all remainign depots
	for(uint i=1;i<num_vehicles_;++i){
		bbox[0] = min(bbox[0],depotPositions_[i][0]);
		bbox[1] = min(bbox[1],depotPositions_[i][1]);
		bbox[2] = max(bbox[2],depotPositions_[i][0]);
		bbox[3] = max(bbox[3],depotPositions_[i][1]);
		
	}
	//add space for safety distance and k-1 vehicle on x axis
	if(safety_distance_ >0 ){
	    bbox[0] -= (num_vehicles_-1) * safety_distance_;
	    bbox[2] += (num_vehicles_-1) * safety_distance_;
	}
	return bbox;
}

void Instance::sort_depots_(){
    //std::vector< std::array<int, 2> > depotPositions_;
    typedef std::array<int, 2> position;
    auto sorter  = [](const position & a, const position & b){ 
        return a[0] < b[0];  };
        
    sort( std::begin(depotPositions_), std::end(depotPositions_), sorter);

}
