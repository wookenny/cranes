#include "InsertionHeuristic.h"
#include "Tours.h"

#include <random>
#include <thread>
#include <tuple>

using namespace std;

typedef tuple<int,int> interval;
	
//some helper functions, forming all the needed intervals for a single 
//job pair

inline void add_int_left(vector<interval>& events, int job_x, int left_job_x, int time, int shift_time){
	int diff_x = left_job_x-job_x;
	cout<< diff_x<<endl;
	if( diff_x > 0 ){
	
		cout<< "["<<time-diff_x<<" - "<<time+diff_x<<"]"<<endl;
		events.push_back( make_tuple(time - diff_x - shift_time,+1) );
		events.push_back( make_tuple(time + diff_x - shift_time,-1) );
	}	
}

inline void add_int_right(vector<interval>& events, int right_job_x, int job_x, int time, int shift_time){
	int diff_x = job_x - right_job_x;
	cout<< diff_x<<endl;
	if( diff_x > 0 ){
	
		cout<< "["<<time-diff_x<<" - "<<time+diff_x<<"]"<<endl;
		events.push_back( make_tuple(time - diff_x - shift_time,+1) );
		events.push_back( make_tuple(time + diff_x - shift_time,-1) );
	}	
}

//ERROR: wenn ich für den neuen Job das ende betrachte, muss ich das
//interval um length() shiften! 

void add_intervals_left_jobs(vector<interval>& events,
							const Job& job, const scheduledJob& leftJob)
{
	//interval: [t_l - (x-x_l), t_l + (x-x_l)] for all 4 pairs of alpha/beta
	int time = get<1>(leftJob); 
	const Job* left = get<0>(leftJob); 
	
	//alpha-alpha
	add_int_left(events, job.alpha()[0], left->alpha()[0],time,0);

	//alpha-beta
	add_int_left(events, job.alpha()[0], left->beta()[0],time+left->length(),0);
	
	//beta-alpha
	add_int_left(events, job.beta()[0], left->alpha()[0],time,job.length());

	//beta-beta
	add_int_left(events, job.beta()[0], left->beta()[0],time+left->length(),job.length());	
}


void add_intervals_right_jobs(vector<interval>& events,
							const Job& job, const scheduledJob& rightJob)
{
	//interval: [t_r - (x_r-x), t_l + (x_r-x)] for all 4 pairs of alpha/beta
	int time = get<1>(rightJob); 
	const Job* right = get<0>(rightJob); 

	//alpha-alpha
	add_int_right(events, right->alpha()[0], job.alpha()[0],time,0);

	//alpha-beta
	add_int_right(events, right->alpha()[0], job.beta()[0],time+right->length(),job.length());
	
	//beta-alpha
	add_int_right(events, right->beta()[0], job.alpha()[0],time,0);

	//beta-beta
	add_int_right(events, right->beta()[0], job.beta()[0],time+right->length(),job.length());	
}

Tours InsertionHeuristic::operator()(const Instance& inst, 
					const vector<uint> &perm, const vector<uint> &assign) const
{
	assert( perm.size() == inst.num_jobs() );
	assert( assign.size() == inst.num_jobs());
	Tours tour(inst.num_vehicles());
	
	//insert every job at the first, collision-free position
	for(uint i=0; i < inst.num_jobs();++i){
		const Job& job = inst[perm[i]];
		int time = earliest_startingtime_(inst,tour,job,assign[i]);	
		cout<<"added job "<<job<<" @ "<< time <<" to vehicle "<<assign[i] <<endl;
		
		tour.add_job(&job, time, assign[i]);
		cout<<"-------------------------------------"<<endl;
	}
	
	
	
	cout<<tour<<endl;
	assert(inst.verify(tour));
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

//TODO:
/*NEXT: local search + parallel methods . aka: threads to evaluate more than 
one perm/assignment at the same time, 
maybe local search

Can I use some SSE/AVX parallization tricks?

*/

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
	
	cout<<" min. time: "<<time<<endl;
	//IF there is a problem -> increase the starting time!
	
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
	

	auto smaller = [] (const interval& i, const interval& j){
		if( get<0>(i) < get<1>(j) )  
			return true;
		if( get<0>(j) < get<1>(i) )
			return false;
		return (get<1>(i) < get<1>(i));
	};
	
	
	//create all intervals
	vector<interval> events;	
	cout<<"jobs on the left"<<endl;
	for(uint i = 0; i <= v; ++i)
		for(uint j=0; j<tours.num_jobs(i); ++j)
			add_intervals_left_jobs(events,job,tours[i][j]);
	cout<<"jobs on the right"<<endl;	
	for(uint i = v; i < tours.num_tours(); ++i)
		for(uint j=0; j<tours.num_jobs(i); ++j)
			add_intervals_right_jobs(events,job,tours[i][j]);
	
	
	//sort
	sort(events.begin(),events.end(),smaller);	
	
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

