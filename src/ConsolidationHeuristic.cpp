#include "ConsolidationHeuristic.h"

#include "InsertionHeuristic.h"
//#include "Superjob.h"

#include <list>

Tours ConsolidationHeuristic::operator()(const Instance& inst) const
{

	if(verbosity_ >0){
		std::cout<<"Solving an instance with jobs:\n";
		for(const auto& j: inst ){
			std::cout<< "job "<<j.num()<<" "<<j<<std::endl;
		}

	}
	//find consolidation
	std::vector<Superjob> con = greedy_consolidation_(inst);
	return operator()(inst,con);
}



Tours ConsolidationHeuristic::operator()(const Instance& inst,
										const std::vector<Superjob>& con) const
{

	//find best tour
	if(verbosity_ >0)
		std::cout<< "Solving TSP to find optimal sequence for superjobs"<<std::endl;
	std::vector<Superjob> sequence = Superjob::best_order(con, inst);

	if(verbosity_ >0)
		std::cout<< "Constr. assigment from sequence to use insertion heur."<<std::endl;

	//insert using the insertion heuristic
 	std::vector<uint> perm;
 	std::vector<uint> assign;
 	for(const Superjob& s: sequence){
 		// get assignment and IDs of jobs, 
 		//sorted by increasing time i.e. starting time
 		std::vector<std::tuple<Job,uint>> sorted = s.get_sorted_assignment();
 		for(auto &e: sorted){
 			Job j;
 			uint veh;
 			std::tie(j,veh) = e;
 			assert(j.num()>=1);
 			assert(j.num()<= static_cast<int>(inst.num_jobs()) );//ID of job
 			assert( std::find(perm.begin(), perm.end(), j.num()-1)==perm.end() );//job no already added
 			assert( veh < inst.num_vehicles() );
 			perm.push_back(j.num()-1);
 			assign.push_back(veh);	
 		}
 	}


    assert(sequence.size() == con.size());
 	assert(perm.size() == inst.num_jobs());
 	assert(assign.size() == inst.num_jobs());
 	//build permutation and assignment



 	InsertionHeuristic heur;
 	return heur(inst,perm, assign);
}

std::vector<Superjob> ConsolidationHeuristic::greedy_consolidation_(const Instance& inst) const
{
	
	std::vector<Superjob> consol;
	std::list<Job> remaining_jobs(inst.begin(),inst.end());
	//diffrent speed classes:
	std::vector<std::tuple<double,double>> speed_bound;
	speed_bound.push_back( std::make_tuple(1, 1)   );
	speed_bound.push_back( std::make_tuple(1,.75) );
	speed_bound.push_back( std::make_tuple(.75,.5) ); 

	
	for(auto e: speed_bound){
		double low,high;
		std::tie(high,low) = e;
		std::vector<Job> pos_jobs;
		std::vector<Job> neg_jobs;
		for(auto it = remaining_jobs.begin();it!=remaining_jobs.end();){
			if(low <= it->x_speed() and it->x_speed() <= high ){
				if(it->is_positive())
					pos_jobs.push_back( *it );
				else
					neg_jobs.push_back( *it );
				it =  remaining_jobs.erase(it);
			}else{
				++it;
			}
		}

		if(verbosity_>0){
			std::cout<<"Building superjobs for "<<pos_jobs.size()<<" positive jobs and ";
			std::cout<< neg_jobs.size()<<" negative jobs with speed "<<low <<" - "<<high<<std::endl;
		}
		std::vector<Superjob> sjobs = build_superjobs_(inst,low,high,pos_jobs,neg_jobs);
		if(verbosity_>0)
			std::cout<< "\t=>  "<<sjobs.size() <<" superjobs."<<std::endl;
		consol.insert(consol.end(), sjobs.begin(), sjobs.end());
	}

	//last jobs: add them as three superjobs: positive, negative, no movement! 
	//important: speed my be 0, don't get confused by that!
	//TODO: think of this
	//Another IDEA: add some with the insertion heuriostic 
	//or split it up and just use
	//the distance.

	assert(remaining_jobs.empty());//IF THIS HAPPENS THEN IT WAS NOT FIXED CORRECTLY


	return consol;
}


std::vector<Superjob> ConsolidationHeuristic::build_superjobs_(
						const Instance& inst,
						double low_speed, double high_speed,
						std::vector<Job>& pos_jobs, 
						std::vector<Job>& neg_jobs) const 
{


	double threshold = .1;//1./inst.num_vehicles();
	std::vector<Superjob> pos_superjobs;
	std::vector<Superjob> neg_superjobs;

	//sort all jobs using their posoitions (alpha in this case)
	auto sort_func = [&](const Job& j1, const Job& j2)
	{
		return  (j1.alpha()[0] < j2.alpha()[0]);  
    };
	std::sort(std::begin(pos_jobs),std::end(pos_jobs), sort_func);
	std::sort(std::begin(neg_jobs),std::end(neg_jobs), sort_func);

	//positive jobs
	for(const Job& j: pos_jobs){
		double best_quality = -1;
		int best_candidate = -1;
		for(uint i=0; i< pos_superjobs.size(); ++i){
			if(pos_superjobs[i].is_addable(j) )
			{
				double quality = pos_superjobs[i].future_quality(j);
				if(quality > best_quality){
					best_quality = quality;
					best_candidate = i;
				}
			}			
		}
		if(verbosity_>0){
			if(best_candidate!=-1){
				std::cout<<"found candidate "<<best_candidate;
				std::cout<<" with quality "<<best_quality<<std::endl;
				
			}
		}
		if(best_quality < threshold or best_candidate==-1){
			Superjob s(inst.num_vehicles(),low_speed,high_speed, true);
			pos_superjobs.push_back(s);
			pos_superjobs.back().add(j);
		}else{
			pos_superjobs[best_candidate].add(j);
		}
	}

	//negative jobs:
	for(const Job& j: neg_jobs){
		double best_quality = 0;
		int best_candidate = -1;
		for(uint i=0; i< neg_superjobs.size(); ++i){
			if(neg_superjobs[i].is_addable(j) )
			{
				double quality = neg_superjobs[i].future_quality(j);
				if(quality > best_quality){
					best_quality = quality;
					best_candidate = i;
				}
			}			
		}
		if(verbosity_>0){
			if(best_candidate!=-1){
				std::cout<<"found candidate "<<best_candidate;
				std::cout<<" with quality "<<best_quality<<std::endl;
				
			}
		}
		if(best_quality < threshold or best_candidate==-1){
			Superjob s(inst.num_vehicles(),low_speed,high_speed, false);
			neg_superjobs.push_back(s);
			neg_superjobs.back().add(j);
		}else{
			neg_superjobs[best_candidate].add(j);
		}
	}


	for(const auto& s: neg_superjobs)
		pos_superjobs.push_back(s);
	//pos_superjobs.insert(pos_superjobs.begin(),neg_superjobs.begin(),
	//											   neg_superjobs.end());
	return pos_superjobs;
}