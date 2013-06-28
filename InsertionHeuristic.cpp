#include "InsertionHeuristic.h"
#include "Tours.h"

#include <random>
#include <thread>


using namespace std;

//TODO:
/*NEXT: local search + parallel methods . aka: threads to evaluate more than 
one perm/assignment at the same time, 
maybe local search

Can I use some SSE/AVX parallization tricks?

*/


Tours InsertionHeuristic::operator()(const Instance& inst, 
					const vector<uint> &perm, const vector<uint> &assign) const
{
	assert( perm.size() == inst.num_jobs() );
	assert( assign.size() == inst.num_jobs());
	
	Tours tour(inst.num_vehicles());
	
	//insert every job at the first, collision-free position
	insertion_helper(inst, perm,assign,tour);
	
	if(local_search_){
		Tours t(inst.num_vehicles());
		bool decrease = get_best_neighbour(inst, perm, assign, t); 
		while(decrease){
			t = Tours(inst.num_vehicles());		
			decrease = get_best_neighbour(inst, perm, assign, t); 
		}
		tour = t;	
	}	
		
	if(debug_) cout<<tour<<endl;

	return tour;
}

Tours InsertionHeuristic::operator()(const Instance& inst) const{
	//randomness
	mt19937 rng; 
	//rng.seed(time(0));

	uint n = inst.num_jobs();
	//get some random permutation
	vector<uint> perm;
	perm.reserve(n);
	for (uint i=0; i<n; ++i) 
		perm.push_back(i);
	random_shuffle(perm.begin(),perm.end()/*,rng*/);
	
	//get some random assignment
	vector<uint> assign;
	uniform_int_distribution<uint> uint_dist(0,inst.num_vehicles()-1);
	for (uint i=0; i<n; ++i) 
		assign.push_back( uint_dist(rng) );
	
	return operator()(inst, perm, assign);
}


bool InsertionHeuristic::get_best_neighbour(const Instance& inst, 
		const std::vector<uint> &perm, const std::vector<uint> &assign, Tours& t) const
{
	bool better_tour = false;
	
	//NEED a reference for best tour!
	Tours original(inst.num_vehicles());
	insertion_helper(inst, perm,assign,original);
	int makespan = inst.makespan(original);
	
	//copy assigment ad permutation to alter them
	vector<uint> p = perm;
	vector<uint> a = assign;
	
	// - swap assign[i] to something else
	for(int i = 0; i<inst.num_jobs(); ++i){
		//0 to v-1
		for(int v = 0; v < assign[i]; ++v){
			a[i] = v;
			//TODO: build tour and compare makespan			
		}
			
		//v+1 to k-1
		for(int v = assign[i]+1; v < inst.num_vehicles(); ++v){
			a[i] = v;
			//TODO: build tour and compare makespan			
		}
		//reset position a[i]
		a[i] = assign[i];
	}
	
	
	// - swap perm[i] with perm[j] 


	return better_tour;							
}

void InsertionHeuristic::insertion_helper(const Instance& inst, const vector<uint> &perm, 			const vector<uint> &assign, Tours &tour) const
{	
	//insert every job at the first, collision-free position
	for(uint i=0; i < inst.num_jobs();++i){
		const Job& job = inst[perm[i]];
		int time = earliest_startingtime_(inst,tour,job,assign[i]);	
		if(debug_) 
			cout<<"added job "<<job<<" @ "<< time <<" to vehicle "<<assign[i] <<endl;
		
		tour.add_job(&job, time, assign[i]);
		if(debug_) 
			cout<<"-------------------------------------"<<endl;
	}
}

uint InsertionHeuristic::earliest_startingtime_(const Instance& inst,
				const Tours& tours,	const Job& job, uint v) const
{
	assert(v < tours.num_tours());
	int time;
	if(tours[v].size() > 0){
			//time from previous job
			auto prevSchedJob = tours[v].back();
			const Job* prevJob = get<0>(prevSchedJob);
			time = get<1>(prevSchedJob) + prevJob->length() +
					dist_inf(prevJob->beta(),job.alpha());
	}else{ 	
		//time from depot
		time = dist_inf( inst.get_depot(v), job.get_alpha());
	}
	
	if(debug_) 
		cout<<" min. time: "<<time<<endl;

	/**
	Algorithm:   
		- create all forbidden intervals 
			for jobs on same or bigger(right) vehicle
		- create all forbidden intervals 
			for jobs on same or smaller(left) vehicle
		- add all intervals as tuple (time,+1/-1) to a vector
		- sort vector(first smaller time,then  -1 < +1)
		- find position where the sum is 0 for the first time via sequential
		  	search
	**/
	
	//create all intervals
	vector<interval> events;	
	if(debug_)  cout<<"jobs on smaller vehicles than job"<<endl;
	for(uint i = 0; i <= v; ++i)
		for(uint j=0; j<tours.num_jobs(i); ++j)
			//if tour[i][j] is right, than job can not be in the right cone of it!
			intervalsForLeftCone(tours[i][j], job, events);
	if(debug_)  cout<<"jobs on bigger vehicles than job"<<endl;	
	for(uint i = v; i < tours.num_tours(); ++i)
		for(uint j=0; j<tours.num_jobs(i); ++j)
			//if tour[i][j] is left, than job can not be in the left cone of it!
			intervalsForRightCone(tours[i][j], job, events);
	
	
	//sort -> using lexicographic order
	sort(events.begin(),events.end());	
	
	//find first pos with sum == 0
	int sum = 0;
	int t = 0;
	for(uint i=0;i<events.size(); ++i){
		sum += get<1>(events[i]);
		t = get<0>(events[i]);
		//found a position where all intervals are closed again
		if(sum == 0 and t >= time)
			return t;
	}
		
	return time;
}


void InsertionHeuristic::intervalsForRightCone(const scheduledJob& rightJob, const Job& job, vector<interval>& events) const
{

	//interval: [t_r - (x_r-x), t_l + (x_r-x)] for all 4 pairs of alpha/beta
	int time = get<1>(rightJob); 
	const Job* right = get<0>(rightJob); 

	//build both tuples
	tuple<int,int> cones[2];
	cones[0] = make_tuple(time, right->alpha()[0]);
	cones[1] = make_tuple(time + right->length(), right->beta()[0]);

	int cone_x, cone_t, diff_x;	

	//j_alpha in a cone1/2?
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = job.alpha()[0] - cone_x;
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x <<" - "<< cone_t + diff_x<<"]"<<endl;
			events.push_back( make_tuple(cone_t - diff_x, +1) );
			events.push_back( make_tuple(cone_t + diff_x, -1) );
		}	
	}
		
	//j_beta in a cone 1/2?
	//=> shift interval with length of job backwards in time
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = job.beta()[0] - cone_x;
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x- job.length() 
				<<" - "<< cone_t + diff_x- job.length()<<"]"<<endl;
			events.push_back( make_tuple(cone_t - diff_x - job.length(), +1) );
			events.push_back( make_tuple(cone_t + diff_x - job.length(), -1) );
		}
	}
}

void InsertionHeuristic::intervalsForLeftCone(const scheduledJob& leftJob, const Job& job, vector<interval>& events) const
{

	//interval: [t_l - (x-x_l), t_l + (x-x_l)] for all 4 pairs of alpha/beta
	int time = get<1>(leftJob); 
	const Job* left = get<0>(leftJob); 

	//build both tuples
	tuple<int,int> cones[2];
	cones[0] = make_tuple(time, left->alpha()[0]);
	cones[1] = make_tuple(time + left->length(), left->beta()[0]);

	int cone_x, cone_t, diff_x;	

	//j_alpha in a cone1/2?
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = cone_x - job.alpha()[0];
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x <<" - "<< cone_t + diff_x<<"]"<<endl;
			events.push_back( make_tuple(cone_t - diff_x, +1) );
			events.push_back( make_tuple(cone_t + diff_x, -1) );
		}	
	}
		
	//j_beta in a cone 1/2?
	//=> shift interval with length of job backwards in time
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = cone_x - job.beta()[0];
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x- job.length() 
				<<" - "<< cone_t + diff_x- job.length()<<"]"<<endl;
			events.push_back( make_tuple(cone_t - diff_x - job.length(), +1) );
			events.push_back( make_tuple(cone_t + diff_x - job.length(), -1) );
		}
	}
}

