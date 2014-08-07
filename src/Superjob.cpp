#include "Superjob.h"

#include <cassert>
#include <array>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

void Superjob::add(const Job& j){
	assert(is_addable(j));
		//add it
	jobs_.push_back(j);
	min_x = std::min(min_x,std::min(j.alpha()[0],j.beta()[0]));
	max_x = std::max(max_x,std::max(j.alpha()[0],j.beta()[0]));

	//increase the job length if executed with the low speed
	jobs_length_ += j.delta_x();
}

double Superjob::future_quality(const Job& j)  const{
	//does not check wehter job is doavle or not
	double l = jobs_length_ + j.delta_x()*speed_low;
	double new_min_x = std::min(min_x,std::min(j.alpha()[0],j.beta()[0]));
	double new_max_x = std::max(max_x,std::max(j.alpha()[0],j.beta()[0]));
	return l/((new_max_x - new_min_x)/speed_low);	
}

int Superjob::distance_to(const Superjob& other) const{
	assert(not empty());
	std::vector<std::array<int,2>> end_pos = end_positions_();
	std::vector<std::array<int,2>> start_pos =  other.start_positions_();
	assert(end_pos.size()==start_pos.size());


	int dist = 0;
	for(uint i=0; i<end_pos.size(); ++i)
		dist = std::max(dist, dist_inf(end_pos[i],start_pos[i]) );
	return dist;
	
}


bool Superjob::cycle_or_long_path_(const std::vector<std::list<uint>> &g,
	std::vector<uint>& in) const {
	assert(g.size()==in.size());
	std::vector<uint> pot(g.size(),0);

	//find elements eith no outgoing arc., start to increase labels from them
	std::queue<uint> current;
	for(uint i=0; i< in.size(); ++i)
		if(in[i]==0)
			current.push(i);

	//cycle <=> current is empty, not all vertices visited!

	//perform breadth first search, using all vertices with no incomming 
	uint counter = 0;
	while(not current.empty() ){
		uint u = current.front();
		current.pop();
		counter +=1 ;
		//use all outging edges
		for(uint v: g[u]){
			//update distance
			assert(pot[u]+1 >= pot[v]); //new valuie is at least as big as prev. value
			pot[v] = pot[u]+1;
			//path with k edges? => k+1 veh. needed
			if(pot[v] >= k_)
				return true;
			in[v] -=1;
			//no remaining ingoing arc?
			if(in[v]==0)
				current.push(v);
		}
	}
	if(counter!=g.size())
		return true;
	return false;
}

bool Superjob::is_addable(const Job& job) const{
	//same direction?
	if( jobs_positive_ and job.is_negative() )
		return false;
	if( (not jobs_positive_) and job.is_positive() )
		return false;

	std::cout<< "job's speed:"<<job.x_speed()<<std::endl;
	std::cout<< job<<std::endl;
	//correct speed? needed: low <= speed <= high
	if(job.x_speed() < speed_low or  job.x_speed() > speed_high)
		return false;

	if(jobs_.empty())
		return true;

	//cycles or path of length > k in set of jobs?	
	//build graph
	std::cout<< "checking graph for adablility" <<std::endl;


	//work on a copy of all these jobs to add thje new job
	std::vector<Job> jobs_copy;
	for(const auto& j: jobs_)
		jobs_copy.push_back(j);
	//adding new job!
	jobs_copy.push_back(job);

	std::vector<uint> in_degree(jobs_copy.size(),0);
	std::vector<std::list<uint>> graph = build_graph_(in_degree,jobs_copy);

	std::cout <<"jobs: "<<jobs_copy.size()<<"\nin degrees:" <<std::endl;
	for(auto e: in_degree){
		std::cout<< e<<" ";
	}
	std::cout<<std::endl;
	std::cout<< "Edges: "<<std::endl;
	for(const auto& l: graph){
		for(auto v: l){
			std::cout<< v<<" ";
		}
		std::cout<<std::endl;
	}
	std::cout<<std::endl;

	//find longest path or cycle => BFS in order of non-labeled incoming arcs
	if (cycle_or_long_path_(graph, in_degree))
		return false;
	return true;
}		


std::vector<std::array<int,2>> Superjob::end_positions_() const{
	std::vector<std::array<int,2>> pos;
	int x = (jobs_positive_)?max_x:min_x;

	std::vector<uint> in_degree(jobs_.size(),0);
	std::vector<std::list<uint>> graph = build_graph_(in_degree,jobs_);
	std::vector<const Job*> extremal = get_extremal_jobs(false,in_degree, graph);

	//find initial y-position
	assert(extremal.size() == k_);
	int last_y_pos=0;
	for(uint p=0;p<k_;++p)
		if(extremal[p]!=nullptr){
			last_y_pos = extremal[p]->beta()[1];
			break;
		}

	for(uint i=0; i<k_;++i){
		if(extremal[i]!=nullptr)
			last_y_pos = extremal[i]->beta()[1];
		pos.push_back(std::array<int,2>{x,last_y_pos});
	}
	return pos;
}


std::vector<std::array<int,2>> Superjob::start_positions_() const{
	std::vector<std::array<int,2>> pos;
	int x = (jobs_positive_)?min_x:max_x;
	std::vector<uint> in_degree(jobs_.size(),0);
	std::vector<std::list<uint>> graph = build_graph_(in_degree,jobs_);
	assert(graph.size() == jobs_.size());
	std::vector<const Job*> extremal = get_extremal_jobs(true, in_degree, graph);

	//find initial y-position
	assert(extremal.size() == k_);
	int last_y_pos=0;
	for(uint p=0;p<k_;++p)
		if(extremal[p]!=nullptr){
			last_y_pos = extremal[p]->alpha()[1];
			break;
		}
	for(uint i=0; i<k_;++i){
		if(extremal[i]!=nullptr)
			last_y_pos = extremal[i]->alpha()[1];
		pos.push_back(std::array<int,2>{x,last_y_pos});
	}
	return pos;
}


std::vector<std::list<uint>> Superjob::build_graph_(std::vector<uint> &in_degree, 
													const std::vector<Job>& jobs) const{
	std::vector<std::list<uint>> graph;
	assert(in_degree.size()==jobs_.size());
	for(uint i=0;i<jobs_.size();++i)
		graph.push_back( std::list<uint>() );

	for(uint i=0; i<graph.size();++i)
		for(uint j: graph[i]){
			const Job* job1 = &jobs[i]; 
			const Job* job2 = &jobs[j]; 
			double time1,time2;
			if(jobs_positive_){
				time1 = (job1->alpha()[0] - min_x)/speed_low;  
				time2 = (job2->alpha()[0] - min_x)/speed_low;
			}else{
				time1 = (max_x - job1->alpha()[0])/speed_low;  
				time2 = (max_x - job2->alpha()[0])/speed_low;
			}

			int order = Job::getOrdering(std::make_tuple(job1,time1),
				std::make_tuple(job2,time2));
			assert(order!=-2);
				if(order==-1){//job1 left of job2
					graph[i].push_back(j);
					in_degree[j]+=1;				
				}else if(order==1){//job1 right of job2
					graph[j].push_back(i);
					in_degree[i]+=1;	
				}
		}
	return graph;
}


std::vector<const Job*> Superjob::get_extremal_jobs(bool starting, 	
						std::vector<uint> &in, 
						const std::vector<std::list<uint>> & g) const
{
	std::vector<int> pot(g.size(),-1);
	assert( jobs_.size()==g.size() );

	//find elements eith no outgoing arc., start to increase labels from them
	std::queue<uint> current;
	for(uint i=0; i< in.size(); ++i)
		if(in[i]==0){
			current.push(i);
			pot[i]=0;
		}

	while(not current.empty() ){
		uint u = current.front();
		current.pop();
		//use all outging edges
		for(uint v: g[u]){
			//update distance
			assert(pot[u]+1 >= pot[v]); //new value is at least as big as prev. value
			pot[v] = pot[u]+1;
			in[v] -= 1;
			//no remaining ingoing arc?
			if(in[v]==0)
				current.push(v);
		}
	}

	//collect jobs for every distance, pot contains a valid assignment
	std::vector<const Job*> extremal_jobs(k_,nullptr);
	for(uint i=0; i<jobs_.size();++i){
		assert(pot[i] < static_cast<int>(k_) );
		assert(pot[i] >= 0 );
		if( extremal_jobs[pot[i]] == nullptr)
			extremal_jobs[pot[i]] = &jobs_[i];
		else{
			if( starting){
				if( extremal_jobs[pot[i]]->alpha()[0] > jobs_[i].alpha()[0]  )
					extremal_jobs[pot[i]] = &jobs_[i];	
			}else{
				if( extremal_jobs[pot[i]]->beta()[0] < jobs_[i].beta()[0]  )
					extremal_jobs[pot[i]] = &jobs_[i];	
			}
		}
	}
	return extremal_jobs;
}


//Finds the best order to execute the superjobs by solving a tsp via LKH
std::vector<Superjob> Superjob::best_order(const std::vector<Superjob>& jobs,
										const Instance& inst)
{
	//write TSP:
	/*
	NAME : 2DVS Instance
	TYPE : ATSP
	DIMENSION: 4
	EDGE_WEIGHT_TYPE : EXPLICIT
	EDGE_WEIGHT_FORMAT :  FULL_MATRIX
	EDGE_WEIGHT_SECTION :
	0 12 100 100
	100 0 3 35
	100 100 0 4
	2 47 9 0
	EOF
	*/
    std::string random_name = boost::filesystem::unique_path("TSP_temp_file_%%%%-%%%%-%%%%-%%%%").native();
    std::string tsp_file     = random_name+".tsp";
    std::string tsp_sol_file = random_name+".sol";

    std::fstream f;
    f.open( tsp_file.c_str(), std::ios::out );
    f << "NAME : 2DVS Instance"<<std::endl;
    //IMPORTANT: This is an asymetric TSP instance
    f << "TYPE : ATSP" <<std::endl;
    f << "DIMENSION: "<< jobs.size()+1 /* add the depot!*/ <<std::endl;
    f << "EDGE_WEIGHT_TYPE : EXPLICIT" <<std::endl;
    f << "EDGE_WEIGHT_FORMAT :  FULL_MATRIX" <<std::endl;
    f << "EDGE_WEIGHT_SECTION :" <<std::endl;
    //fist: depot line
    f<<"0 ";
    //write dist from depot to job begin
    for(const auto& j1: jobs){
    	f<< j1.distance_from_depot(inst)<< " ";
    }
    f<<std::endl;

    for(const auto& j1: jobs){
    	//dist from j1 to depots
    	f<< j1.distance_to_depot(inst)<< " ";
        for(const auto& j2: jobs){
        	if(&j1 == &j2)//same adress?	
        		f<< "0 ";
            else 
            	f<< j1.distance_to(j2)<< " ";
	   	}
	   	f<<std::endl;
    } 
    f << "EOF" <<std::endl;
    f.close();

    std::vector<Superjob> sequence;
	//syscall LKH
    int ret = system( ("../LKH-2.0.6/test/LKH "+tsp_file+" "+tsp_sol_file).c_str() );
    	//+" &> /dev/null").c_str() );
    if(ret!=0){
    	assert(false);
    	std::remove( tsp_file.c_str() );
    	return sequence;
    }

	//parse solution
	/*
	NAME : 2DVS.21.tour
	COMMENT : Length = 21
	COMMENT : Found by LKH [Keld Helsgaun] Tue Aug  5 16:58:48 2014
	TYPE : TOUR
	DIMENSION : 4
	TOUR_SECTION
	1
	2
	3
	4
	-1
	EOF
	*/
	std::fstream fin;
    fin.open(  tsp_sol_file.c_str(), std::ios::in );
    std::string line;
    while(line!="TOUR_SECTION"){
        getline(fin, line);
    }

    //assertion: first index in tour is 1, which is the depot    
	bool vertices_visited = 0;
	while(getline(fin, line)){  //line after tour section.
		if(line == "-1" or line =="EOF") 
			break;
		int index = std::stoi(line)-2;//depot is index 0
		if(index == -1){
			//if this might not be true: std::rotate om vector!
			assert(vertices_visited == 0);
			++vertices_visited;
			continue;		
		}
		++vertices_visited;		
		assert(index>=0 and index <= static_cast<int>(jobs.size()) );
		sequence.push_back( jobs[index] );
	}

    //delete both files
   /* if( std::remove( tsp_file.c_str() ) != 0 )
	    std::cerr<<"WARNING: Could not delete"<<tsp_file<<"!"<< std::endl;
	if( std::remove( tsp_sol_file.c_str() ) != 0 )
	    std::cerr<<"WARNING: Could not delete"<<tsp_sol_file<<"!"<< std::endl;
	*/
	return sequence;
}


int Superjob::distance_to_depot(const Instance& inst) const{
	//to dept <=> distance between end of superjob and depot
	std::vector<std::array<int,2>> ends = end_positions_();
	int dist = 0;
	for(uint i = 0; i<k_; ++i){
		dist = std::max(dist, dist_inf(inst. get_depot(i), ends[i]) );
	}
	return dist ;
}


int Superjob::distance_from_depot(const Instance& inst) const{
	//from dept <=> distance between depot and start of superjob
	std::vector<std::array<int,2>> starts = start_positions_();
	int dist = 0;
	for(uint i = 0;i<k_; ++i){
		dist = std::max(dist, dist_inf(inst.get_depot(i), starts[i]) );
	}
	return dist ;
}
	

std::vector<std::tuple<Job,uint>> Superjob::get_sorted_assignment() const{

	//sort the jobs
	std::vector<Job> jobs_copy = jobs_;
	//lambda to sort, depemds if Superjobs is positive or negative
	auto sort_func = [&](const Job& j1, const Job& j2) -> bool
	{
		if(jobs_positive_)
			return  (j1.alpha()[0] < j2.alpha()[0]);  
		else
			return  (j1.alpha()[0] > j2.alpha()[0]);  
   };

	std::sort(jobs_copy.begin(), jobs_copy.end(), sort_func);
	
	//get the assigment => maybe extract a metjhod: get assignment
	std::vector<uint> in(jobs_copy.size(),0);
	std::vector<std::list<uint>> graph = build_graph_(in,jobs_copy);
	std::vector<int> pot(graph.size(),-1);
	std::queue<uint> current;
	for(uint i=0; i< in.size(); ++i)
		if(in[i]==0){
			current.push(i);
			pot[i]=0;
		}

	while(not current.empty() ){
		uint u = current.front();
		current.pop();
		//use all outging edges
		for(uint v: graph[u]){
			//update distance
			assert(pot[u]+1 >= pot[v]); //new value is at least as big as prev. value
			pot[v] = pot[u]+1;
			in[v] -= 1;
			//no remaining ingoing arc?
			if(in[v]==0)
				current.push(v);
		}
	}

	//put tuples into a set and return it
	std::vector<std::tuple<Job,uint>> sorted_assignment;
	for(uint i=0; i < jobs_copy.size(); ++i){
		assert(pot[i]>=0);
		sorted_assignment.push_back( std::make_tuple(jobs_copy[i],pot[i]) );
	} 

	assert(sorted_assignment.size() == jobs_.size());
	return sorted_assignment;
}


//------ Here are some GTests for this class---//
#ifdef GTESTS_ENABLED
#include <gtest/gtest.h>
TEST(Superjob_Tests, Addable_Test) { 
   
    Superjob s(2, .5, 1, true);

   	//3 vehivles, speed between 0.5 and 1, all jobs positive
    //Job(int num,int alpha_x,int alpha_y,int beta_x, int beta_y)
    Job j1(1, 0,0, 10,0);
    Job j2(2, 2,1, 12,1);

    Job j3(3, 5,0, 14,0); 
    Job j4(4, 13,0, 12,0);
    Job j5(5, 30,1, 31,5); 
    EXPECT_TRUE(s.is_addable(j1));
    s.add(j1);

    EXPECT_DOUBLE_EQ( .5,s.quality());

    EXPECT_TRUE(s.is_addable(j2));
    //s.add(j2);

    //EXPECT_DOUBLE_EQ(24/22.,s.quality());

    EXPECT_FALSE(s.is_addable(j3));// at leat 3 vehicles needed
    EXPECT_FALSE(s.is_addable(j4));//negative job
    EXPECT_FALSE(s.is_addable(j5));//speed is too low(.25)


}



#else

#endif