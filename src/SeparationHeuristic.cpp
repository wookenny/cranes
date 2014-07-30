#include "SeparationHeuristic.h"
#include "Tours.h"
#include "Common.h"
#include "SingleCraneTSP_Solver.h"

#include <list>
#include <set>
#include <unordered_map>

using namespace std;

bool schedule_initial_subset(Tours& , const Instance&, 
                             const std::vector<Job>& , uint,
                             int, int,
                             std::unordered_map<int, std::array<int, 2>>&, 
                             std::unordered_map<int, double>&, double );
bool schedule_subset(Tours&, const Instance&, 
                     const std::vector<Job>&, uint, uint,
                     std::unordered_map<int, std::array<int, 2>>& , 
                     std::unordered_map<int, double>&, double );


Tours SeparationHeuristic::operator()(const Instance& inst) const{
	Tours tours(inst.num_vehicles());


	const uint k = inst.num_vehicles();
	double best_makespan = std::numeric_limits<double>::infinity();

	//collect all positions and test all variations
	std::set<int> x_positions;

	for(const auto& job: inst){
		x_positions.insert( job.get_alpha()[0] );
		x_positions.insert( job.get_beta()[0]  );
	}


	//set -> vector to allow a reordering of the elements 
	std::vector<int> positions( begin(x_positions), end(x_positions) );
    //TODO: a shuffle might speed up things!!!
    std::sort( begin(positions), end(positions) );

	//iterate over all combinations
    uint counter = 0;
    double makespan = std::numeric_limits<double>::infinity();
    //calculate an initiall solution -> "nice" separation
    std::vector<int> vec1,vec2;
    for(uint i=1; i<k;++i){
        vec1.push_back( positions[ std::floor(i*(1.*positions.size()/k)) ] );
        vec2.push_back( positions.front()+i*(positions.back()-positions.back())/k);
    }

    Tours init_t1 = operator()(inst, vec1);
    Tours init_t2 = operator()(inst, vec2);
    assert(inst.verify(init_t1));
    assert(inst.verify(init_t2));
    best_makespan =  inst.makespan(init_t1);
    tours = init_t1;
    if( inst.makespan(init_t2) < best_makespan ){
         best_makespan =  inst.makespan(init_t2);
         tours = init_t2;
    }
    if(verbosity_>0) std::cout<<"starting with inti. sol: "
                             <<best_makespan<<std::endl; 
    if(stop_after_initial_) return tours;
                         
	do{
			++counter;
            //slice the first k-1 elements of the positions
            std::vector<int> v(begin(positions), begin(positions) +(k-1));

    		Tours t = operator()(inst, v, makespan);
    		// solutionm might be empty , due to early abort
            if(not t.empty()){
                assert(inst.verify(t));
                makespan = inst.makespan(t);
        		if(makespan < best_makespan){
        			best_makespan = makespan;
        			tours = t;
                    if(verbosity_ > 0)
                        std::cout<<"makespan: "<< makespan
                             <<" test("<<counter<<")"<<std::endl;
        		}else
                    if(verbosity_>1) std::cout<<"step "<<counter
                                    <<": no impovement"<<std::endl;  
            }else{
                if(verbosity_>1) std::cout<<"step "<<counter
                                <<": early aborted"<<std::endl;  
            }

    }
    while( next_combination( begin(positions),begin(positions) + (k-1),
                             end(positions)) );
    if(verbosity_ > 0)
        std::cout<<"configurations: "<<counter<<std::endl;
	return tours;
}


Tours SeparationHeuristic::operator()(const Instance& inst, 
									  std::vector<int>& pos,
                                      double bound ) const
{
	//Hint: Depots do not need to be inside the boundary given by the positions
	std::sort(begin(pos), end(pos));

	const uint k = inst.num_vehicles();
    auto box = inst.get_bounding_box();
    int min_x = box[0];
    int max_x = box[2];
    assert(box[0] <= box[2]);

	Tours tours(k);

    std::list<Job> unplanned(begin(inst), end(inst));
    pos.insert(std::begin(pos),(min_x)); //there is no push_front
    pos.push_back(max_x);

    std::unordered_map<int, std::array<int, 2>>  veh_pos;
    std::unordered_map<int, double>              veh_time;
    for(uint v = 0; v<k; ++v){
        veh_pos[v] = inst.get_depot(v); 
        veh_time[v] = 0;
    }


    int step = 1;
    while(not unplanned.empty()){
        //plan jobs, remove finished jobs and execute jobs
        for(uint i=0; i<k; i+=step){
            int left_border  = pos[i];
            int right_border = (pos.size()>i+step)?pos[i+step]:pos.back();

            if( left_border > right_border )
                std::cout<< left_border<<"-"<<right_border <<std::endl;
            assert(left_border <= right_border);
            //collect jobs contained in interval
            std::vector<Job> jobs;
            for (auto it = unplanned.begin(); it != unplanned.end(); /*no ++it */){
                //erasing an elements increments the iteratpor to the next position
                if( it->contained_in_xrange(left_border, right_border) ){
                    jobs.push_back(*it);
                    it = unplanned.erase(it);
                }
                else{
                    ++it;
                }
            }

            bool below_bound;
            if(step==1)
                below_bound = schedule_initial_subset(tours, inst, jobs, i, 
                                left_border, right_border, veh_pos, veh_time, bound);
                //std::cout<<" "<<contained<<" jobs + "<<to_add<<" -> "<<tours.num_jobs()<<std::endl; 
            else
                below_bound = schedule_subset(tours, inst, jobs, i, 
                                    std::min(i+step-1,k-1), veh_pos, veh_time, bound);
            if(not below_bound)
                return Tours(inst.num_vehicles());
        }
        //assert that all positions are valid all the time!
        for(uint i=0; i<inst.num_vehicles()-1;++i)
            assert(veh_pos[i][0] <= veh_pos[i+1][0] );
        step*=2;
    }

    if(not inst.verify(tours)){
        inst.debug(true);
        inst.verify(tours);
        std::cout<< tours<<"\n\n"<<std::endl;
    }
    assert(inst.verify(tours));
	return tours;
}

bool schedule_initial_subset(Tours& tours, const Instance& inst, 
                     const std::vector<Job>& jobs, uint v,
                     int left_border, int right_border,
                     std::unordered_map<int, std::array<int, 2>>& veh_pos, 
                     std::unordered_map<int, double>& veh_time, double bound)
{ 
    assert(veh_time[v]==0);
    if(jobs.size()==0){
        //adjust position and starting time if needed
        if( veh_pos[v][0]<left_border ){
            veh_time[v] = left_border-veh_pos[v][0];
            veh_pos[v][0] = left_border;
        }else if( right_border < veh_pos[v][0] ){
            veh_time[v] = veh_pos[v][0]-right_border;
            veh_pos[v][0] = right_border;
        }
        return (veh_time[v]<bound);
    }

    Instance subinst(1);
    subinst.add_depotposition(veh_pos[v]);
    for(auto j: jobs)
        subinst.add_job(j);

    //solve via TSP: 
    SingleCraneTSP_Solver solver;
    solver.set_verbosity(-1);
    auto sol = solver(subinst);
    Tours t = get<1>( sol );

    //add jobs to the solution
    assert(t.num_jobs()==jobs.size());
    for(uint i=0; i<t.num_jobs();++i){
        const Job* job;
        double starting;
        std::tie(job, starting) = t[0][i];
        uint ID = job->num();
        const Job* job_ptr = nullptr;
        if( ID-1 < inst.num_jobs())
            job_ptr = &inst[ID-1];

        //wrong job? => seach for the job by iteration
        if( *job_ptr!= *job){
            for(uint j=0; j < inst.num_jobs(); ++j){
                if( inst[j] == *job){
                    job_ptr = & inst[j];  
                    break;
                }
            }
        }
        assert(job_ptr!=nullptr);
        tours.add_job(job_ptr, starting, v);

        //update vehicle time and date
        if(starting+job->length() > veh_time[v]){
            veh_time[v] = starting+job->length();
            veh_pos[v] = job->beta();
        } 
        //early abort?
        if(starting+job->length() > bound)
            return false;
            
    }
    tours.sort_jobs();
    return true;
}  

bool schedule_subset(Tours& tours, const Instance& inst, 
                     const std::vector<Job>& jobs, uint v_begin, uint v_end,
                     std::unordered_map<int, std::array<int, 2>>& veh_pos, 
                     std::unordered_map<int, double>& veh_time, double bound)
{ 
    if(jobs.size()==0)
        return true;
    
    Instance subinst(1);
    for(auto j: jobs)
        subinst.add_job(j);

    //test all possible vehicles, select the best. 
    double best_makespan = std::numeric_limits<double>::infinity();
    uint best_vehicle = v_begin;
    double offset = 0; 
    Tours best_tour(1);

    //test all possible positions with the specific offset, 
    //such there is no collision 
    for(uint v = v_begin; v<=v_end; ++v){      
        //don't check a position twice
        if (v>v_begin and veh_pos[v]==veh_pos[v-1])
            continue;

        subinst.clear_depots();
        subinst.add_depotposition( veh_pos[v] );
        
       double curr_offset = veh_time[v];
        for(uint i = v_begin; i<=v_end; ++i){
            if(i==v) continue;
            curr_offset = max(curr_offset,
                              veh_time[i]-abs(veh_pos[v][0]-veh_pos[i][0]) );
        }

        //solve the subprob
        SingleCraneTSP_Solver solver;
        solver.set_verbosity(-1);
        auto sol = solver(subinst);
        Tours t = get<1>( sol );
        double objective = get<0>( sol );

        //better solution?
        if(curr_offset + objective < best_makespan){
            best_makespan = curr_offset + objective;    
            best_vehicle = v;
            offset = curr_offset;
            best_tour = t;
        }
    }        
    assert( best_tour.num_jobs() == jobs.size());
    //augment the solution
    for(uint i=0; i<best_tour.num_jobs();++i){
        const Job* job;
        double starting;
        std::tie(job, starting) = best_tour[0][i];
        uint ID = job->num();
        const Job* job_ptr = nullptr;
        if( ID-1 < inst.num_jobs())
            job_ptr = &inst[ID-1];

        //wrong job? => seach for the job by iteration
        if( *job_ptr!= *job){
            for(uint j=0; j < inst.num_jobs(); ++j){
                if( inst[j] == *job){
                    job_ptr = & inst[j];  
                    break;
                }
            }
        }
        assert(job_ptr!=nullptr);
        tours.add_job(job_ptr, starting+offset, best_vehicle); 
    }
    tours.sort_jobs();

    //Update vehicle position and time
    const Job* job;
    double starting;
    std::tie(job, starting) = tours[best_vehicle].back();    
    auto end_pos = job->beta();
    bool driven_left = (end_pos[0] < veh_pos[best_vehicle][0]);
    //copy old position
    veh_pos[best_vehicle] = end_pos;
    veh_time[best_vehicle] = starting + job->length();
    if(veh_time[best_vehicle]>bound)
        return false;
    //std::cout<<"fixing range "<<v_begin<<" "<<v_end<<std::endl;
    if(driven_left){
        //correct all veh. on the left
        for(uint w = best_vehicle-1; w>=v_begin; --w){
            //std::cout<<"fixing depot "<<w<<std::endl;
            if( veh_pos[w][0] > end_pos[0]){
                veh_pos[w][0] = end_pos[0];
                assert(veh_time[w]<=veh_time[best_vehicle]);
                veh_time[w]=veh_time[best_vehicle];
            }else{
                break;
            }
        }        
    }else{
         //correct all veh. on the right
        for(uint w = best_vehicle+1; w<=v_end; ++w){
            //std::cout<<"fixing depot "<<w<<std::endl;
            if( veh_pos[w][0] < end_pos[0]){
                veh_pos[w][0] = end_pos[0];
                assert(veh_time[w]<=veh_time[best_vehicle]);
                veh_time[w]=veh_time[best_vehicle];
            }else{
                break;
            }
        }        
    }
    return true;
}
