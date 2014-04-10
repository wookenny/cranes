#include "MipInsertionHeuristic.h"
#include "Common.h"
#include "independent_TSP_MIP.h" 
#include <iostream>

using std::cout;
using std::endl;
using std::boolalpha;


Tours MipInsertionHeuristic::operator()(const Instance& inst, 
										const std::vector<uint>& p) const {

	std::vector<uint> perm = p;
	if(verbosity_>0){
		cout<<"Permutation: "<<endl;
		for(auto elem: perm)
			cout<<elem<< " ";
		cout<<endl;	 
	}	

	Tours tour(inst.num_vehicles());
	auto time_remaining = getTimer(time_limit_); 
	if(local_search_){
		Tours t(inst.num_vehicles());	
		bool decrease =  get_better_neighbour(inst, perm, t); 
		while(decrease){
			cout<<"Found: "<<inst.makespan(t)<<endl;
			decrease =  get_better_neighbour(inst, perm, t); 
			assert(inst.verify(t));	
			if(not time_remaining()) 
				break; 
		}
		insertion_helper(inst, perm,tour);	
	}else{
		insertion_helper(inst, perm, tour);
	}	
			
	if(debug_) cout<<tour<<endl;
	
	return tour;
}

Tours MipInsertionHeuristic::operator()(const Instance& inst) const {
	auto perm = random_permutation(inst.num_jobs(), seed_);
	assert( perm.size() == inst.num_jobs() );
	return operator()(inst, perm);
}

bool MipInsertionHeuristic::get_better_neighbour(const Instance& inst, 
								std::vector<uint>& perm, Tours& tour) const {
    tour.clear();
	insertion_helper(inst, perm,tour);
	int makespan = inst.makespan(tour);

	for(uint i=0; i < perm.size(); ++i){
		//find partners in smaller groups
		uint smallest_in_cluster = (i/cluster_size_)*cluster_size_;
		uint largest_in_cluster  =  smallest_in_cluster + cluster_size_;
		/*for(uint j = 0; j < smallest_in_cluster; ++j ){
			//smap end evaluate
			std::swap(perm[i],perm[j]);
			tour.clear();
			insertion_helper(inst, perm,tour);
			if(inst.makespan(tour) < makespan){
				return true;
			}
			std::swap(perm[i],perm[j]);
		}*/

		for(uint j = largest_in_cluster+1; j < perm.size(); ++j ){
			//smap end evaluate
			std::swap(perm[i],perm[j]);
			tour.clear();
			insertion_helper(inst, perm,tour);
			if(inst.makespan(tour) < makespan){
				return true;
			}
			std::swap(perm[i],perm[j]);
		}
	}

	assert(inst.verify(tour));
    return false;
}


std::vector<std::array<int, 2>> last_depot_Positions(Tours& t,const Instance & inst){
	std::vector< std::array<int, 2> > positions;
	t.sort_jobs();
	for(uint i=0; i < inst.num_vehicles(); ++i){
		std::array<int, 2> pos;
		if (t[i].empty()){
			pos = inst.get_depot(i);
		}
		else{
			const Job* job;
			std::tie(job,std::ignore) = t[i].back();
			pos = job->beta();
		}
		positions.push_back(pos);
	}

	//fix the order
	for(uint i=inst.num_vehicles()-1; i >=1; --i){
		positions[i][0] = std::max(positions[i][0],positions[i-1][0]);
	}

	return positions;
}

double get_makespan_for_depot(const Tours& t, 
								std::vector<std::array<int, 2>> depots){
	double time = 0;
	for(uint i=0; i < t.num_tours(); ++i){
		if (t[i].empty()) continue;
		const Job* job;
		double start;
		std::tie(job,start) = t[i].back();
		start += job->length() + dist_inf(job->beta(),depots[i]);
		time = std::max(time, start);		
	}
	return time;
}

void MipInsertionHeuristic::insertion_helper(const Instance& inst, 
											const std::vector<uint> &perm, 			
                                            Tours& tour) const {
		tour.clear();
		double offset = 0;
		//save intial depot positions,
		//later: retrieve them from current vehicle position
		std::vector< std::array<int, 2> > depotPositions;
		for(uint d=0; d < inst.num_vehicles(); ++d)
			depotPositions.push_back(inst.get_depot(d));
		

		for(uint pos = 0; pos <  inst.num_jobs(); pos+= cluster_size_){
			//build open end mip
			if(not local_search_)
				cout<< 100*pos/inst.num_jobs()<<"% done"<<endl;
			Instance small = inst.splice(pos, cluster_size_,perm);
			for(uint d=0; d <depotPositions.size(); ++d)
				small.set_depotposition(depotPositions[d],d);

			auto mip = independent_TSP_MIP(small);
			mip.set_collision(true);
			mip.set_LP(false);
			mip.set_returning_to_depot(false);
			if(verbosity_<2)
				mip.set_silent(true);

			//solve mip and add to solution
			double makespan;
			Tours small_tour(small.num_jobs());
			std::tie(small_tour, makespan) = mip.solve(); 

			//add jobs to tour 
			auto schedule = small_tour.get_schedule(); 
        	//type: (std::map<int,std::tuple<Job, double, int>> )
        	//ID -> Job, starting time, vehicle
			for (auto kv : schedule) {
			 	int ID = kv.first;
			 	double startingtime;
			 	int veh;
			 	const Job* const job = &inst[ID-1];
			 	assert(*job == std::get<0>(kv.second));
			 	std::tie(std::ignore,startingtime,veh) = kv.second;	
 			   	tour.add_job(job, offset + startingtime, veh);
			}
			//store changed values
			depotPositions = last_depot_Positions(small_tour,small);
			double add_time = get_makespan_for_depot(small_tour, depotPositions);
			
			assert(add_time + 0.01 >= makespan );
			offset += add_time;
		}
		tour.sort_jobs();
		assert(inst.verify(tour));
}	


