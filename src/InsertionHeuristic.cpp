#include "InsertionHeuristic.h"
#include "Tours.h"
#include "Common.h"
#include <thread>
#include <random>
#include <thread>
#include <utility>
#include <chrono>

using namespace std;
    
//%TODO: use safety distance in this methods!

//------------------------------------------------------------------//
//     Helper class defining swaps in assignment and permutation    //
//------------------------------------------------------------------//
class LocalSearch_Swap{
    public:
        LocalSearch_Swap(const Instance& i, std::vector<uint> p, 
                            std::vector<uint> a, int t, int tt, int &o, 
                            std::vector<uint> &bp,std::vector<uint> &ba,
                            bool better=true,bool* s=nullptr):
                            inst(i),perm(p),assign(a),thread(t),
                            total_threads(tt),opt(o),best_perm(bp),
                            best_assign(ba),find_better(better),stop(s){}
        void operator()()
	    {       
            InsertionHeuristic insert;
            Tours tour{0};
            
            if(use_assign)
                tour = insert(inst, perm, assign);
            else
                tour = insert(inst, perm);   
			opt = inst.makespan(tour);

            //two exchange in permutation
            for(uint i = thread; i<inst.num_jobs(); i+=total_threads){
        		for(uint j = i+1; j<inst.num_jobs(); ++j){
			        swap(perm[i],perm[j]);		
			        if(stop!=nullptr and *stop) return;
			        //another thread found a solution and this one can stop?
			        bool is_better = evaluate(insert, tour);
    		        if(is_better and find_better){
        	            if(stop!=nullptr and not *stop){
                        //set to stop, if pointer given and still false
				            *stop = true;
				            return;
				        }
			        }		
			        //reswap:
			        swap(perm[i],perm[j]);
		        }
        	}
        	
        	//change in assignment
        	if(not use_assign) return;
        	//cout<<"looking for better assignment"<<endl;
        	for(uint i = thread; i<inst.num_jobs(); i+=total_threads){
                uint orig_vehicle = assign[i];
                for(uint k=0;k < inst.num_vehicles();++k){
        	        if(k==orig_vehicle) continue;
        	        assign[i] = k;
        	        if(stop!=nullptr and *stop) return;
			        //another thread found a solution and this one can stop?
        	        bool is_better = evaluate(insert, tour);
    		        if(is_better and find_better){
        	            if(stop!=nullptr and not *stop){
                        //set to stop, if pointer given and still false
				            *stop = true;
				            return;
				        }
			        }		
        	    }
        	    assign[i] = orig_vehicle;
        	    //reset originally assigned vehicle   
        	}     	  
        }
        
        void set_use_assign(bool b){use_assign = b;}
        
    private:
        const Instance& inst;
        std::vector<uint> perm;
        std::vector<uint> assign;
        int thread;
        int total_threads;
        int& opt;
        std::vector<uint> &best_perm;
        std::vector<uint> &best_assign;          
        bool find_better;
        bool* stop;
        bool use_assign = false;
        
        bool evaluate(InsertionHeuristic& insert, Tours & tour){	
	        tour.clear();
	        if(use_assign)
	             tour = insert(inst, perm, assign);
	        else tour = insert(inst, perm); 
		    int new_makespan = inst.makespan(tour);
		    if( new_makespan < opt){
		        opt = new_makespan;
		        //cout<<"t_"<<thread<<": "<<opt<<endl;
				best_perm = perm;
				if(use_assign)
				    best_assign = assign;	    
				return true;
			}
			return false;	        
	    } 		
};

//------------------------------------------------//
//     Method of the Class Insertion Heuristic    //
//------------------------------------------------//

/* Main method/functor. Generates an assignment and a permutation 
and starts the insertion heuristic.*/
Tours InsertionHeuristic::operator()(const Instance& inst) const{

    safety_distance_ = inst.get_safety_distance();

    //measure the time    
    using namespace std::chrono;
    starting_time_ = system_clock::now();

	//randomness
	mt19937 rng; 
	rng.seed(seed_);

	uint n = inst.num_jobs();
	//get some random permutation
	vector<uint> perm;
	perm.reserve(n);
	for (uint i=0; i<n; ++i) 
		perm.push_back(i);
	vector<uint> assign(0);	
    uniform_int_distribution<uint> uint_distr(0,inst.num_vehicles()-1);
	
	Tours solution(inst.num_vehicles());
	uint runs = 0;
	bool ls = local_search_;
	local_search_ = false;
	bool first_run = true;
	//merge first runs and the last runs with additional localsearch
	while(runs < runs_ + random_starts_){
	    if(runs == random_starts_)
	        local_search_ = ls;
		//randomize permutation and assignment
		if(!first_run){
		    random_shuffle(perm.begin(),perm.end());
		}else{
		    first_run = false;
		}  
		if(no_assignment == false){  
		    assign.clear();
		    for (uint i=0; i<n; ++i) 
			    assign.push_back( uint_distr(rng) );  
		}
		//construct tour with ass. + perm	
		Tours t{static_cast<int>(inst.num_vehicles())};
		if(no_assignment)
		    t = operator()(inst, perm);
		else t = operator()(inst, perm, assign);
		if(0==runs or  inst.makespan(t)<inst.makespan(solution))
			solution = t;
		++runs;
			
		if (not time_remaining()){
		    cout<<"Timelimit of "<< time_limit_<<" seconds reached. Stopping after "<<runs;
		    cout<<(runs>=2?" runs\n":" run.\n");
		    break;
		 }   
	}
	
	if(verbosity_>0){
	    auto now = std::chrono::system_clock::now();        
        //auto total_seconds = duration_cast<seconds>(now - starting_time_);
	    cout<<"Running time: "<< duration_to_string(now - starting_time_) <<endl;
	 }
	//cout<< "best: "<<inst.makespan(solution)<<endl;
	assert(inst.verify(solution));	
	return solution;
}


/* insertion heuristic with given permutation, but without an assignment*/
Tours InsertionHeuristic::operator()(const Instance& inst, 
					const vector<uint> &perm) const
{
    
	assert( perm.size() == inst.num_jobs() );
	
	safety_distance_ = inst.get_safety_distance();
	Tours tour(inst.num_vehicles());

	vector<uint> p = perm;
    vector<uint> a;
    assert(a.size()==0); 
	assert(p.size()==inst.num_jobs());
	if(local_search_){
		Tours t(inst.num_vehicles());	
		
		bool decrease =  get_better_neighbour_parallel(inst, p, a, t); 
		while(decrease){
			 decrease =  get_better_neighbour_parallel(inst, p, a, t); 
			 assert(inst.verify(t));	
			 if(not time_remaining()) break; 
		}
		insertion_helper(inst, p,tour);	
	}else{
		insertion_helper(inst, p,tour);
	}	
			
	if(debug_) cout<<tour<<endl;
	  
	return tour;
}

/* insertion heuristic with given permutation and assignment*/
Tours InsertionHeuristic::operator()(const Instance& inst, 
					const vector<uint> &perm, const vector<uint> &assign) const
{
	assert( perm.size() == inst.num_jobs() );
	assert( assign.size() == inst.num_jobs() or assign.size()==0);
	
	safety_distance_ = inst.get_safety_distance();
	Tours tour(inst.num_vehicles());

	vector<uint> p = perm;
	vector<uint> a = assign;
	static int initial_runs_no_assign = static_cast<int>(.1 * inst.num_jobs());
 	assert(p.size()==a.size());
 	assert(p.size()==inst.num_jobs());
 	vector<uint> empty_a;
 	
	if(local_search_){
		Tours t{static_cast<int>(inst.num_vehicles())};	
		bool decrease;
		if(initial_runs_no_assign >0)
		     decrease =  get_better_neighbour_parallel(inst, p, empty_a, t);
		else decrease =  get_better_neighbour_parallel(inst, p, a, t);
		
		while(decrease or initial_runs_no_assign > 0){
    	    if(not time_remaining()) break; 
			if(initial_runs_no_assign <=0){
			    decrease =  get_better_neighbour_parallel(inst, p, a, t); 
			    assert(inst.verify(t));	 
            }else{
                decrease =  get_better_neighbour_parallel(inst, p, empty_a, t); 
                assert(inst.verify(t));	
		        --initial_runs_no_assign;
                if(initial_runs_no_assign==0){
                    if(verbosity_>0)
                        cout<< "copying assignment and using it from now on"<<endl;
                    auto schedule = t.get_schedule();
                    for(int i=0; i< static_cast<int>(p.size()); ++i){
                        assert(schedule.count(p[i]+1)>0);    
                        a[i] = std::get<2>(schedule[p[i]+1]);
                    }
                }    
		    }
	    }	
		
		insertion_helper(inst, p,a,tour);  	
		assert(inst.verify(tour));	
	}else{
		insertion_helper(inst, p,a,tour);
		assert(inst.verify(tour));	
	}	

	if(debug_) cout<<tour<<endl;
	return tour;
}

bool InsertionHeuristic::get_better_neighbour_parallel(const Instance& inst, 
		std::vector<uint> &perm, std::vector<uint> &assign,Tours& t) const
{
    //palce to store if a thread is already done
    bool halt_threads = false;
    //find best or better argument in neughborhood?

	//need a reference for best tour!
    bool found_better_tour = false;
	Tours best_tour(inst.num_vehicles());
	if(assign.size()>0)
	     insertion_helper(inst, perm,assign,best_tour);
	else insertion_helper(inst, perm,best_tour);
	int makespan = inst.makespan(best_tour);

    const int num_threads = (threads_>0)?threads_:thread::hardware_concurrency();
    vector<std::thread> threads;
    vector<int> opt(num_threads);
    vector<vector<uint>> best_perms(num_threads);
    vector<vector<uint>> best_assign(num_threads);
    
    
    //Launch a group of threads
    for (int i = 0; i < num_threads; ++i){
    
        auto lswap = LocalSearch_Swap(inst,perm,assign,i,num_threads,
                                      opt.at(i), best_perms.at(i),
                                      best_assign.at(i),stop_at_better_,
                                      &halt_threads );

        lswap.set_use_assign( assign.size()>0 );
        threads.push_back( std::thread(lswap) );
    }
    //wait for them
    for(auto &t: threads)
        t.join();

    //find min
    int best_index = -1;
    for(int i=0; i<num_threads; ++i){
        if(opt[i]<makespan){
            assert(is_permutation(best_perms[i]));
            found_better_tour = true;
            makespan = opt[i];
            best_index = i;
        }
    }

    if(best_index>=0){
        perm    = best_perms[best_index];
        assign  = best_assign[best_index];
        assert(is_permutation(perm));
        t.clear(); 
        if(assign.size()==0)
             insertion_helper(inst, perm,t);
        else insertion_helper(inst, perm,assign,t);
        if(verbosity_>0) cout<<"found: "<<makespan << endl;
        assert( makespan == inst.makespan(t) );
        //if(verbosity_>0) cout<<"assign len: "<<assign.size()<< endl;
    }    
    return found_better_tour;
}


void InsertionHeuristic::insertion_helper(const Instance& inst, 
                                          const vector<uint> &perm, 			
                                          const vector<uint> &assign, 
                                          Tours &tour) const
{	
    assert(perm.size()==inst.num_jobs());
    assert(assign.size()==inst.num_jobs());
    assert( is_permutation(perm) );
	//insert every job at the first, collision-free position
	
	for(uint i=0; i < inst.num_jobs();++i){
		const Job& job = inst[perm[i]];
		int time = earliest_startingtime_(inst,tour,job,assign[i]);	
		if(debug_) 
			cout<<"added job "<<job<<" @ "
			    << time <<" to vehicle "<<assign[i] <<endl;
					
		tour.add_job(&job, time, assign[i]);
		if(debug_) 
			cout<<"-------------------------------------"<<endl;
	}
}


void InsertionHeuristic::insertion_helper(const Instance& inst, 
                                          const vector<uint> &perm, 			
                                           Tours &tour) const
{	
	//insert every job at the first, collision-free position
	for(uint i=0; i < inst.num_jobs();++i){
		const Job& job = inst[perm[i]];
		int time = earliest_startingtime_(inst,tour,job,0);
		uint assign = 0;
		for(uint v=1; v<inst.num_vehicles();++v){
		    int curr_time  = earliest_startingtime_(inst,tour,job,v);	
		    if(curr_time< time){
		        time = curr_time;
		        assign = v;
		    }
		}    
		if(debug_) 
			cout<<"added job "<<job<<" @ "
			    << time <<" to vehicle "<<assign <<endl;
		
		tour.add_job(&job, time, assign);
		if(debug_) 
			cout<<"-------------------------------------"<<endl;
	}
}

uint InsertionHeuristic::earliest_startingtime_(const Instance& inst,
				const Tours& tours,	const Job& job, uint v) const
{
    if(v >= tours.num_tours())
        cout<<"insert into tour "<<v<<", number veh.:  "<<tours.num_tours() <<endl;
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
			(shift right job (diff_vehicle)*safety dist to the left)
		- create all forbidden intervals 
			for jobs on same or smaller(left) vehicle
			(shift current job (diff_vehicle)*safety dist to the left)
		- add all intervals as tuple (time,+1/-1) to a vector
		- sort vector(first smaller time,then  -1 < +1)
		- find position where the sum is 0 for the first time via sequential
		  	search
	**/
	
	//TODO: add method to use safety distance to this thingy
	
	//create all intervals
	vector<interval> events;	
	if(debug_)  cout<<"jobs on smaller vehicles than job"<<endl;
	for(uint i = 0; i <= v; ++i)
		for(uint j=0; j<tours.num_jobs(i); ++j)
		//if tour[i][j] is right, than job can not be in the right cone of it!
			intervalsForLeftCone(tours[i][j], i, job, v, events);
	if(debug_)  cout<<"jobs on bigger vehicles than job"<<endl;	
	for(uint i = v; i < tours.num_tours(); ++i)
		for(uint j=0; j<tours.num_jobs(i); ++j)
		//if tour[i][j] is left, than job can not be in the left cone of it!
			intervalsForRightCone(tours[i][j], i, job, v, events);
	
	
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


void InsertionHeuristic::intervalsForRightCone(const scheduledJob& rightJob, 
                                               uint v_right,
                                               const Job& job, uint v_left, 
                                               vector<interval>& events) const
{

	//interval: [t_r - (x_r-x), t_l + (x_r-x)] for all 4 pairs of alpha/beta
	int time = get<1>(rightJob); 
	const Job* right = get<0>(rightJob); 

	//build both tuples
	tuple<int,int> cones[2];
	int offset = (v_right - v_left) * safety_distance_;
	assert(offset >= 0);
	cones[0] = make_tuple(time, right->alpha()[0] - offset );
	cones[1] = make_tuple(time + right->length(), right->beta()[0] - offset);

	int cone_x, cone_t, diff_x;	

	//j_alpha in a cone1/2?
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = job.alpha()[0] - cone_x;
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x <<" - "
				    << cone_t + diff_x<<"]"<<endl;
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

void InsertionHeuristic::intervalsForLeftCone(const scheduledJob& leftJob, 
                                              uint v_left,
                                              const Job& job, uint v_right,
                                              vector<interval>& events) const
{

	//interval: [t_l - (x-x_l), t_l + (x-x_l)] for all 4 pairs of alpha/beta
	int time = get<1>(leftJob); 
	const Job* left = get<0>(leftJob); 

	//build both tuples
	tuple<int,int> cones[2];
	int offset = (v_right - v_left) * safety_distance_;
	assert(offset >= 0);
	cones[0] = make_tuple(time, left->alpha()[0]);
	cones[1] = make_tuple(time + left->length(), left->beta()[0]);

	int cone_x, cone_t, diff_x;	

	//j_alpha in a cone1/2?
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = cone_x - (job.alpha()[0] - offset);
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x <<" - "
				    << cone_t + diff_x<<"]"<<endl;
			events.push_back( make_tuple(cone_t - diff_x, +1) );
			events.push_back( make_tuple(cone_t + diff_x, -1) );
		}	
	}
		
	//j_beta in a cone 1/2?
	//=> shift interval with length of job backwards in time
	for( auto cone : cones){
		tie(cone_t, cone_x) = cone;
		diff_x = cone_x - (job.beta()[0]-offset);
		if(diff_x > 0){
			if(debug_) 
				cout<< "["<< cone_t - diff_x- job.length() 
				<<" - "<< cone_t + diff_x- job.length()<<"]"<<endl;
			events.push_back( make_tuple(cone_t - diff_x - job.length(), +1) );
			events.push_back( make_tuple(cone_t + diff_x - job.length(), -1) );
		}
	}
}

bool InsertionHeuristic::time_remaining() const{
    using namespace std::chrono;
    if(time_limit_ <=0)
        return true;    
    
    auto now = std::chrono::system_clock::now();        
    auto total_seconds = duration_cast<seconds>(now - starting_time_);
    return (total_seconds.count() < time_limit_);
}

