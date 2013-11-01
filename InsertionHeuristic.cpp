#include "InsertionHeuristic.h"
#include "Tours.h"
#include <thread>
#include <random>
#include <thread>
#include <utility>

using namespace std;

//TODO: change parallel mode to deal with swaps
//TODO: change parallel mode to find better neighbor

bool is_permutation(vector<uint> v){
    sort(begin(v),end(v));
    for(uint i=0; i<v.size();++i)
        if(v[i]!=i)
            return false;
    return true; 
}

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
            Tours tour = insert(inst, perm, assign);
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
        
        bool evaluate(InsertionHeuristic& insert, Tours & tour){	
	        tour.clear();
	        tour = insert(inst, perm, assign);
		    int new_makespan = inst.makespan(tour);
		    if( new_makespan < opt){
		        opt = new_makespan;
		        //cout<<"t_"<<thread<<": "<<opt<<endl;
				best_perm = perm;
				best_assign = assign;
				return true;
			}
			return false;	        
	    } 		
};


Tours InsertionHeuristic::operator()(const Instance& inst, 
					const vector<uint> &perm, const vector<uint> &assign) const
{
	assert( perm.size() == inst.num_jobs() );
	assert( assign.size() == inst.num_jobs());
	
	Tours tour(inst.num_vehicles());

	vector<uint> p = perm;
	vector<uint> a = assign;
	
	
	if(local_search_){
		Tours t(inst.num_vehicles());	
		bool decrease =  get_best_neighbour_parallel(inst, p, a, t); 
		//bool decrease =  get_better_neighbour(inst, p, a, t); 
		while(decrease)
			 decrease =  get_best_neighbour_parallel(inst, p, a, t); 
			 //decrease =  get_better_neighbour(inst, p, a, t); 

		insertion_helper(inst, p,a,tour);	
	}else{
		insertion_helper(inst, p,a,tour);
	}	
		
		
	if(debug_) cout<<tour<<endl;

	return tour;
}

Tours InsertionHeuristic::operator()(const Instance& inst) const{
	

	//randomness
	mt19937 rng; 
	rng.seed(time(0));

	uint n = inst.num_jobs();
	//get some random permutation
	vector<uint> perm;
	perm.reserve(n);
	for (uint i=0; i<n; ++i) 
		perm.push_back(i);
	vector<uint> assign;	
	uniform_int_distribution<uint> uint_dist(0,inst.num_vehicles()-1);
	
	Tours solution(inst.num_vehicles());
	uint runs = 0;
	bool ls = local_search_;
	local_search_ = false;
	//merge first runs and the last runs with additional localsearch
	while(runs < runs_ + random_starts_){
	    if(runs == random_starts_)
	        local_search_ = ls;
		//randomize permutation and assignment
		random_shuffle(perm.begin(),perm.end());
		assign.clear();
		for (uint i=0; i<n; ++i) 
			assign.push_back( uint_dist(rng) );
		
		Tours t = operator()(inst, perm, assign);
		if(0==runs or  inst.makespan(t)<inst.makespan(solution))
			solution = t;
		++runs;
		//cout<<"-------------------" <<endl;
	}
	//cout<< "best: "<<inst.makespan(solution)<<endl;	
	return solution;
}


bool InsertionHeuristic::get_better_neighbour(const Instance& inst, 
		std::vector<uint> &perm, std::vector<uint> &assign, Tours& t) const
{
	
	//need a reference for best tour!
	Tours best_tour(inst.num_vehicles());
	insertion_helper(inst, perm,assign,best_tour);
	int makespan = inst.makespan(best_tour);
	
	//copy assigment ad permutation to alter them
	
	//cout<< "local search: "<<makespan<<endl;
	
	Tours tour(inst.num_vehicles());
	// - swap assign[i] to something else
	for(uint i = 0; i<inst.num_jobs(); ++i){
		uint tmp = assign[i];
	
		//0 to v-1
		for(uint v = 0; v < tmp; ++v){
			assign[i] = v;
			//build tour and compare makespan
			tour.clear();
			insertion_helper(inst, perm,assign,tour);
			int new_makespan = inst.makespan(tour);
			if(new_makespan < makespan){
				makespan = new_makespan;
		
				//return !
				t = tour;
				cout<< makespan<< endl;
				return true;				
			}
			
		}
			
		//v+1 to k-1
		for(uint v = tmp+1; v < inst.num_vehicles(); ++v){
			assign[i] = v;
			//build tour and compare makespan	
			tour.clear();
			insertion_helper(inst, perm,assign,tour);
			int new_makespan = inst.makespan(tour);
			if(new_makespan < makespan){
				makespan = new_makespan;
				cout<< makespan<< endl;
			    return true;
			}		
		}
		//reset position a[i]
		assign[i] = tmp;
	}
	
	
	// - swap perm[i] with perm[j] 
	for(uint i = 0; i<inst.num_jobs(); ++i){
		for(uint j = i+1; j<inst.num_jobs(); ++j){
			swap(perm[i],perm[j]);
					
			//build tour and compare makespan	
			tour.clear();
			insertion_helper(inst, perm,assign,tour);
			int new_makespan = inst.makespan(tour);
			if(new_makespan < makespan){
				makespan = new_makespan;
				cout<< makespan<< endl;
				return true;
			}		
			//reswap:
			swap(perm[i],perm[j]);
		}
	}
	return false;							
}


bool InsertionHeuristic::get_best_neighbour_parallel(const Instance& inst, 
		std::vector<uint> &perm, std::vector<uint> &assign,Tours& t) const
{
    //palce to store if a thread is already done
    bool halt_threads = false;
    //find best or better argument in neughborhood?
    bool find_better  = true;

	//need a reference for best tour!
    bool found_better_tour = false;
	Tours best_tour(inst.num_vehicles());
	insertion_helper(inst, perm,assign,best_tour);
	int makespan = inst.makespan(best_tour);

    const int num_threads = (threads_>0)?threads_:thread::hardware_concurrency();
    vector<std::thread> threads;
    vector<int> opt(num_threads);
    vector<vector<uint>> best_perms(num_threads);
    vector<vector<uint>> best_assign(num_threads);
    
    
    //Launch a group of threads
    for (int i = 0; i < num_threads; ++i)
        threads.push_back( 
            std::thread( LocalSearch_Swap(inst,perm,assign,i,num_threads,
                 opt.at(i), best_perms.at(i),best_assign.at(i),find_better,&halt_threads ))
            );
    
    //wait for them
    for(auto &t: threads)
        t.join();

    //find min
    int best_index = -1;
    for(int i=0; i<num_threads; ++i){
        if(opt[i]<makespan){
            //assert(is_permutation(best_perms[i]));
            found_better_tour = true;
            makespan = opt[i];
            best_index = i;
        }
    }

    if(best_index>=0){
        perm    = best_perms[best_index];
        assign  = best_assign[best_index];
        //assert(is_permutation(perm));
        t.clear(); 
        insertion_helper(inst, perm,assign,t);
        cout<<"found: "<<makespan << endl;
    }    
    return found_better_tour;
}


void InsertionHeuristic::insertion_helper(const Instance& inst, 
                                          const vector<uint> &perm, 			
                                          const vector<uint> &assign, 
                                          Tours &tour) const
{	
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


void InsertionHeuristic::intervalsForRightCone(const scheduledJob& rightJob, 
                                               const Job& job, 
                                               vector<interval>& events) const
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
                                              const Job& job, 
                                              vector<interval>& events) const
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

