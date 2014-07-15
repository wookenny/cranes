#include "independent_TSP_MIP.h"
#include "DisjointSet.h"
#include "Common.h"
#include "MinCut.h"

#include <set>
#include <thread>
#include <algorithm>
using namespace std;


 //template function for a nicer lookup function
 template<template<class,class> class CONTAINER, typename E, typename A >
 bool contains(const CONTAINER<E, A >& container, const E& needle){
     return find(container.begin(), container.end(), needle) != container.end();
 }
  
 template<template<class,class,class> class CONTAINER, typename E, typename A , typename O>
 bool contains(const CONTAINER<E, O, A >& container, const E& needle){
     return find(container.begin(), container.end(), needle) != container.end();
 }
   
   
void independent_TSP_MIP::add_objective_function_(){
       model_.add(IloMinimize(env_,vars_[v_["makespan"]]));
} 
 
//building variables
void independent_TSP_MIP::build_variables_(){
	bool use_assign = (assignment_.size() > 0);
	assert(assignment_.size()==0 or assignment_.size()==inst_.num_jobs());

	auto type = IloNumVar::Bool;
	if(LP_relaxation_)
		type = IloNumVar::Float; 

    string name;
	//additional depot and 
	name = "makespan";
	v_[name] = counter_++;

	if(fixed_makespan_ <= 0)	
	    vars_.add( IloNumVar(env_, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
    else
	    vars_.add( IloNumVar(env_,fixed_makespan_, fixed_makespan_,IloNumVar::Float, name.c_str() ) );
    
    //time variables
	for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
		name = name_t_(j);
		v_[name] = counter_++;
		vars_.add( IloNumVar(env_, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
	}
	
	//directed graph variables
    for(uint i = 1; i <= inst_.num_jobs()+inst_.num_vehicles();++i)
		for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
			//skip variable x_i_i, if not a depot
			if(i==j and i<=inst_.num_jobs() )
				continue;
			//skip inter depot variables
			if(i>inst_.num_jobs() and j>inst_.num_jobs())
				continue;	
 			for(uint v = 1; v <= inst_.num_vehicles();++v){			
 				//no variables from/to depot i with another vehicle than i
 				if(i>inst_.num_jobs() and i-inst_.num_jobs()!=v)
 					continue;
 				if(j>inst_.num_jobs() and j-inst_.num_jobs()!=v)
 					continue;
				
				string name = name_x_(i,j,v);
				v_[name] = counter_++;
				int lb = 0; int ub = 1;
				if(use_assign){
					//do not use edge if not both jobs are assigned to v
					if (i <= inst_.num_vehicles() and assignment_[i-1]+1 != v){ 
						ub = 0;
					}	
					if (j <= inst_.num_vehicles() and assignment_[j-1]+1 != v){
						ub = 0;
					}
					//for i,j > k: other job is real and assign == v
				}
				vars_.add( IloNumVar(env_, lb, ub, type, name.c_str() ) );
			}

		}

	//directed graph variables for depot loops
	for(uint i = inst_.num_jobs()+1; i <= inst_.num_jobs()+inst_.num_vehicles();++i){
		assert(i-inst_.num_jobs()>0);
		string name = name_x_(i,i,i-inst_.num_jobs());
		v_[name] = counter_++;  
		vars_.add( IloNumVar(env_, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
	}
	
		
	//tour variable:
	for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
		name = name_k_(j);
		v_[name] = counter_++;
		auto var_type = IloNumVar::Int;
		if(LP_relaxation_)
			var_type = IloNumVar::Float;
		int lb = 1;
		int ub = inst_.num_vehicles();

		if (use_assign and  j <= inst_.num_jobs() ){
			assert(assignment_[j-1] < inst_.num_vehicles());
			lb = ub = assignment_[j-1]+1;
		}
		vars_.add( IloNumVar(env_, lb, ub, var_type, name.c_str() ) );
	}	
	
		
}

										
//constraint construction
void independent_TSP_MIP::build_constraints_(){
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	
	if (bigM <= 0)
	    bigM = inst_.get_upper_bound();
    if(fixed_makespan_>0)
        bigM = fixed_makespan_;
    if( not start_.empty() and assignment_.empty() )
    	bigM = inst_.makespan(start_);
	//indegree == outdegree nodes (single vehicle flow)
	for(uint v=1; v<=K;++v){
		for(uint i = 1; i<=n+K; ++i){
			if(i>n and v!=i-n) //constraint with veh. i for depot j
					continue;
		
			IloExpr expr(env_);
			//outgoing edges(from i)
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and i>n and i!=j) continue; //no interdepot constraint,
													//except selfloop
				if(i==j and j<= n) continue;//no selfloop, except depots
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;

				expr += x(i,j,v);
			}
			//incoming edges(from i)
			for(uint j = 1; j<=n+K; ++j){
				if(i==j and j<= n) continue;//no selfloop, except depots
				if(j>n and i>n and i!=j) continue; //no interdepot constraint,
													//except selfloop
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;

				expr -= x(j,i,v);
			}

			IloRange constraint(env_, 0, expr, 0,
			("in_outdegree_for_"+to_string(i)+"_vehicle_"+to_string(v)).c_str());
			cons_.add(constraint);		
		}
	}
			
	//add IN degree for vertices (combined flow)
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint v = 1; v<=K; ++v){
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;
				if(i>n and v!=i-n) //no edges from depot with wrong vehicle 
					continue;
	 			if(i==j and i<=n) continue; //add x_ii only for depot
				expr += x(j,i,v);
			}
		}

		IloRange constraint(env_, 1, expr, 1, 
							("in_degree_for_"+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
		
	//add OUT degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint v = 1; v<=K; ++v){
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;
				if(i>n and v!=i-n) //no edges from depot with wrong vehicle 
					continue;
	 			if(i==j and i<=n) continue; //add x_ii only for depot
				expr += x(i,j,v);
			}
		}

		IloRange constraint(env_, 1, expr, 1, 
							("out_degree_for_"+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
	

	//add inflow = outflow for all subgraphs k
	for(uint i = 1; i<=n+K; ++i){
		for(uint v = 1; v<=K; ++v){
			IloExpr expr(env_);
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;
				if(i>n and v!=i-n) //no edges from depot with wrong vehicle 
					continue;
	 			if(i==j and i<=n) continue; //add x_ii only for depot
				expr += x(i,j,v);
				expr -= x(j,i,v);
			}
			IloRange constraint(env_, 0, expr, 0, 
							("subflow_conservation_"+to_string(v)+"_vertex_"
													+to_string(i)).c_str() );
			cons_.add(constraint);	
		}		
	}


	//starting time variables for depots
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( t(i) == 0);	
	
	
	//starting time variables for regular jobs
	for(uint i = 1; i<=n; ++i)
		for(uint j = 1; j<=n; ++j){
			if(i==j) continue;
			IloExpr expr(env_);
			const Job& job_i = inst_[i-1];
			const Job& job_j = inst_[j-1];
			///*
			auto tmpBigM = bigM+ job_i.length() + dist_inf(job_i.beta(),job_j.alpha());
			expr += 1*t(j);
			expr -= 1*t(i);
			for(uint v =1; v<=K;++v)
				expr -= tmpBigM*x(i,j,v);
			IloRange constraint(env_, 
				job_i.length() + dist_inf(job_i.beta(),job_j.alpha()) -tmpBigM, 
				expr, IloInfinity, 
				("increasing_starting_time_"+
					to_string(i)+"_"+to_string(j)).c_str() );
			cons_.add(constraint);	
			//add them multiple times
			for(uint v =1; v<=K;++v){
				cons_.add( t(j) - 1*t(i)- tmpBigM*x(i,j,v)  
							>=  job_i.length() + dist_inf(job_i.beta(),job_j.alpha())
								 -tmpBigM);
			}
			
		}
	
	//starting time variables for "first" jobs after depots
	for(uint i = n+1; i<=n+K; ++i)
		for(uint j = 1; j<=n; ++j){
			if(i==j) continue;
			const Job& job_j = inst_[j-1];
			cons_.add( t(j)  - dist_inf(inst_.get_depot(i-n-1),job_j.alpha()) * x(i,j,i-n) >=0 );
		}	

	//makespan constraint for every job
	for(uint i=1; i<=n; ++i){
		//IloNumVar &makespan = vars_[v_["makespan"]];
		const Job& job = inst_[i-1];
		IloExpr expr(env_);
		if (fixed_makespan_<=0)
			expr += 1* vars_[v_["makespan"]];
		else
		    expr += fixed_makespan_;
		expr -= t(i);
		expr -= job.length();
		//add length to choosen depot
		if(returning_to_depot_)
			for(uint v = 1; v<=K; ++v)
				expr -= dist_inf(inst_.get_depot(v-1),job.get_beta()) * x(i,n+v,v);
			
		IloRange constraint(env_, 0, expr, IloInfinity, ("makespan_cons_due_to_vehicle_"+to_string(i)).c_str() );
		cons_.add(constraint);	
	}


	//vehicle number for depots is their number
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( k(i) == i-n);
	
		
	//executing vehicle defined by the used edges!
	for(uint i = 1; i<=n; ++i){
		IloExpr expr(env_);
		expr +=  k(i);
		for(uint v=1; v<=K;++v){
			for(uint j = 1; j<=n+K; ++j){
				if(i==j) continue;	//do not respect selfloops
				if(j>n and j-n!=v)  //no edges from depot with wrong vehicle
					continue;
				expr -=  static_cast<IloInt>(v)* x(j,i,v);
			}
		}
		IloRange constraint(env_, 0, expr, 0, 
				("vehicle_assignment_"+to_string(i)).c_str() );
		cons_.add(constraint);
	}
	
	//additional constraint to increase the makespan:
	//makespan >= each single tour
	if(tighten_){
		for(uint v=1; v<=K;++v){
				IloExpr expr(env_);
				expr =  1.0 * vars_[v_["makespan"]];
				//inter job vertices
				for(uint i = 1; i<=n; ++i)
			        for(uint j = 1; j<=n; ++j){
			            if(i==j) continue;
			            const Job& j1 = inst_[i-1];
			            const Job& j2 = inst_[j-1];
			            int l = j1.length();
			            l += dist_inf(j1.beta(),j2.alpha());
			            
			            expr -=  l* x(i,j,v);
			        }
			    //depot->job vertices, job->depot vertices        
				for(uint i = 1; i<=n; ++i){
	                    //to depot:
						const Job& j1 = inst_[i-1];
			            int l = j1.length();
			            //do not drive back to depot if not needed!
			            if(returning_to_depot_){
			            	l += dist_inf(j1.beta(),inst_.get_depot(v-1));	            
			            	expr -=  l * x(i,n+v,v);
			        	}
			            //from depot
			            l = dist_inf(inst_.get_depot(v-1),j1.alpha());
			            expr -=  l * x(n+v,i,v);
			    }
				    
				IloRange constraint(env_, 0, expr, IloInfinity,
				("length_bound_for_tour_"+to_string(v)).c_str());
				cons_.add(constraint);		
		}
	}

	
	
}

	
//parsing of solution		
void independent_TSP_MIP::parse_solution_(Tours &tours){
	//check wheter the model is capable of producing falid sol
	if( not collision_avoidance_ or LP_relaxation_)
		return;

	uint n = inst_.num_jobs();

	tours.clear();
	//build all k tours
	for( uint j = 1; j<=n; ++j){
		//int v = std::round(cplex_.getValue(k(j)));
		int v = cplex_.getIntValue(k(j));
		assert( v - cplex_.getIntValue(k(j)) < 0.1);
		assert(v > 0);
		assert(static_cast<uint>(v) <= inst_.num_vehicles() );
		double time = cplex_.getValue(t(j));
		tours.add_job(&inst_[j-1], time, v-1);
	}
	tours.sort_jobs();

	//cplex_.exportModel("mTSP.lp");
	//cplex_.writeSolutions("mTSP.sol"); 
}	



//Cut callback 
//checks for fractional subtours, which means: ?????
ILOUSERCUTCALLBACK1(SubtourCutsCallback, independent_TSP_MIP*, mip )
{   

	// mip->x(j,i,i-n)    
 	// auto arc = [&] (int i, int j, int k){ return xSol[mip->v_[mip->name_x_(i,j,k)]];};
	//arcs: 

   // Skip the separation if not at the end of the cut loop
   // Skip the separation if not at the end of the cut loop
   if( !isAfterCutLoop() )
      return;
    
 
	 
	uint n = mip->inst_.num_jobs();
 	uint K = mip->inst_.num_vehicles(); 

	//parse LP solution   
	IloEnv env = getEnv();
	IloInt numNodes =  mip->vars_.getSize();
	IloNumArray xSol(env, numNodes);
	getValues(xSol,  mip->vars_);

	auto arc = [&] (int i, int j, int k){ return xSol[mip->v_[mip->name_x_(i,j,k)]];};
	//calculate min cut
	std::vector<std::tuple<int,int,double>> edges;
	//capacity of 2 between depots
	for(uint i=n+1; i<=n+K; ++i)
		for(uint j=i+1; j<=n+K; ++j){
			edges.push_back(make_tuple(i-1,j-1,2));
		}
	//ask once for every variable!	
	//add sum of arce in both dorections
	for(uint i=1; i<=n; ++i){
		for(uint j=i+1; j<=n+K;++j){
			double c = 0;
			if(j>n){//depot
				c+= arc(i,j,j-n) + arc(j,i,j-n);
			}else{//two normal jobs
				for(uint v=1;v<=K;v++)
					c += arc(i,j,v) + arc(j,i,v);
			}
			if(c> 0.0001)	
				edges.push_back(make_tuple(i-1,j-1,c));
		}
	}
	std::vector<bool> cut = find_min_cut(edges,n+K); 

    assert(cut.size()==n+K or cut.size()==0);

    if(cut.size()==0){
    	xSol.end();
    	return;
    }
	IloExpr new_cut(env);

	for(uint i=1; i<=cut.size(); ++i)
		for(uint j=i+1; j<=cut.size(); ++j)
			if(cut[i-1] != cut[j-1]){
				//std::cout<< "add undir. arc "<<i<<"-"<<j <<std::endl;
				assert( i<=n or j<=n);//not two depots!
				if(i>n){//i is a depot vertex
					new_cut += 1*mip->x(i,j,i-n);
					new_cut += 1*mip->x(j,i,i-n);
				}else if(j>n){// j is a depot
					new_cut += 1*mip->x(i,j,j-n);
					new_cut += 1*mip->x(j,i,j-n);
				}else{//two non-depot vertices
					for(uint v=1;v<=K;++v){
						new_cut += 1*mip->x(i,j,v);
						new_cut += 1*mip->x(j,i,v);
					}
				}

			}
	add(new_cut >= 2).end();
	new_cut.end();
   //free all cplex stuff
   xSol.end();
   return;


}


/**
 Own solve()-method to enable an own cut-callback
**/

std::pair<Tours,double> independent_TSP_MIP::solve(){
	auto start = std::chrono::system_clock::now();
	Tours tours(inst_.num_vehicles());

	//build the MIP model and solve it!
	try {
	
		if(silent_)
			cplex_.setParam(IloCplex::MIPDisplay, 0);

		if(timelimit_ > 0)
			cplex_.setParam(IloCplex::TiLim, 60*timelimit_);

		build_variables_();
		
		if( 1 != inst_.num_vehicles() and  collision_avoidance_ )
			build_collision_variables_();
		
		model_.add(vars_);
			
		//objective function: minimize makespan
		if(fixed_makespan_ < 0)
		    add_objective_function_();
		
		build_constraints_();
	
		if( 1!= inst_.num_vehicles() and  collision_avoidance_)
			build_collision_constraints_();  

		model_.add(cons_);

       
		//run cplex and solve the model!
		cplex_.extract(model_);
		
		//write model to file
		//TODO: remove
		//cplex_.exportModel("tmp_mip.lp");
		
	    //add MIP start
        add_MIP_start_();
		
		//print small models 
		if( debug_ and inst_.num_jobs()<=6)
			cout<<"MIP Model: \n"<<model_<<endl;
		
		//add callback to create cuts on the fly
		uint threads = thread::hardware_concurrency();
		if(use_subtour_cuts_)
			cplex_.use( SubtourCutsCallback(env_, this) );
		cplex_.setParam(IloCplex::IntParam::Threads	,threads);
		if(not silent_) cout<<"Detected "<<threads<<" cores" <<endl;
		
		if(not silent_ and debug_ and fixed_makespan_>0)
		    cout<< "Checking for solution with makespan at most "
		        << fixed_makespan_<< endl;
			
		bool solved = cplex_.solve();
		if(debug_) cout<< "Solving MIP was successful: "<<boolalpha<<solved<<endl;
		if(debug_) cout<< "MIP status = "<<cplex_.getStatus()<<endl;
		auto stop = std::chrono::system_clock::now();
		runningtime_ = std::chrono::duration_cast<std::chrono::seconds>(stop - start);		if ( !solved ) {
			env_.end();
			//return empty tours if MIP was unsolvable!
			return pair<Tours,double>{Tours(inst_.num_vehicles()),-1};
		}else{
			//parse solution
			//if(LP_relaxation_ and inst_.num_jobs()<10)
			//	print_LP_solution_();
			
			found_objective_ =  cplex_.getObjValue();
			parse_solution_(tours);
		}
		
		if(debug_)	print_LP_solution_();
		
	}catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "WARNING: Unknown exception caught while handling with CPLEX" << endl;
		
		exit(1); // Returns 1 to the operating system
	}
	//end it
	env_.end();


	return pair<Tours,double>(tours,found_objective_);
}


void independent_TSP_MIP::add_MIP_start_(){
    if(LP_relaxation_ or start_.empty() ) return;
    if(fixed_makespan_>0) return; //not sound to set a start in this setting
    
    auto schedule = start_.get_schedule();

    uint n = inst_.num_jobs();
 	uint K = inst_.num_vehicles();
        
    IloNumVarArray startVar(env_);
    IloNumArray startVal(env_);

    //makespan,	a variable for the makespan
    startVar.add(vars_[v_["makespan"]]);
    startVal.add(inst_.makespan(start_));
      
    //t_j, 		starting times for all jobs and ll depots
    for(uint i=1; i<=n; ++i){
        startVar.add(t(i));
        //starting time shopuld be an integer! or at least close to it
        assert( fabs(get<1>(schedule[i])- (int)(get<1>(schedule[i])))<.01 );       
        startVal.add( (int)(get<1>(schedule[i])) );
    }

    //x_i,j,k for used edges between jobs
    for(uint v=0; v< K; ++v)
        for(uint i=0; i+1 < start_[v].size(); ++i){
            int job1 = std::get<0>(start_[v][i])->num();
            int job2 = std::get<0>(start_[v][i+1])->num();
                           
            startVar.add(x(job1,job2,v+1));
            startVal.add( 1 );
        }

    //edges from depot to first job AND from last to depot
    for(uint v=0; v< K; ++v){
        if(start_[v].empty()) continue;
    
         int first_job = std::get<0>(start_[v].front())->num();
         int last_job = std::get<0>(start_[v].back())->num();
         
         //depot to first job
         startVar.add(x(n+v+1,first_job,v+1));
         startVal.add( 1 );
            
         //last job to depot   
         startVar.add(x(last_job,n+v+1,v+1));
         startVal.add( 1 );   
    }        
       
    
    
    //k_j,  \in (1,...,k),tour variable, which assigns a job to a vehicle	
    for(uint i=1; i<=n; ++i){
        startVar.add(k(i));
        startVal.add(get<2>(schedule[i])+1);
    }
   
    //k_j for depots:
    for(uint i=1; i<=K; ++i){
        startVar.add(k(n+i));
        startVal.add(i);
    }
   
    //add variables used for the collisions, if they are used 
    if(collision_avoidance_){     
		std::set<string> variables;
		
		for(uint i=1; i<=n; ++i){
		    for(uint j=1; j<=n; ++j){
			    if(i==j) continue;
			
			    const Job& job_i = inst_[i-1];
			    const Job& job_j = inst_[j-1];
			    auto alpha_x = [&](const Job& job)->int{return job.get_alpha()[0];};
			    auto beta_x  = [&](const Job& job)->int{return job.get_beta()[0];};
			    
			    int t_i =  (int)(get<1>(schedule[i])); 
			    int t_j =  (int)(get<1>(schedule[j]));
			    
		        //PLUS VARIABLES
		        //alpha_x(job_j) - alpha_x(job_i) - t(j) + t(i) >0 => caa_p(i,j) = 1
		        if( alpha_x(job_j) - alpha_x(job_i) - t_j + t_i > 0.0001 ){
		             startVar.add(caa_p(i,j));
		             startVal.add(1);
		             variables.insert(name_caa_p_(i,j));
		        }
		        //beta_x(job_j) - job_j.length() 
				// - alpha_x(job_i))  - t(j) + t(i) >0 => cab_p(i,j) = 1
		        if( beta_x(job_j) - job_j.length() - alpha_x(job_i)
		              - t_j + t_i > 0.0001 )
		        {
		            startVar.add(cab_p(i,j));
		            startVal.add(1);
		            variables.insert(name_cab_p_(i,j));
		        }
		        
		        //alpha_x(job_j) - beta_x(job_i) + job_i.length() 
		        //- t(j) + t(i)                  >0 => cba_p(i,j) = 1
		        if( alpha_x(job_j) - beta_x(job_i) + job_i.length() 
		                - t_j + t_i > 0.0001 )
		        {
		            startVar.add(cba_p(i,j));
		            startVal.add(1);
		            variables.insert(name_cba_p_(i,j));
		        }
		        
		        //beta_x(job_j) -job_j.length() 
		        // -beta_x(job_i) + job_i.length() - beta_x(job_i) + job_i.length() 
		        // - t(j) + t(i) > 0 => cbb_p(i,j) = 1 
                if( beta_x(job_j) - job_j.length() - beta_x(job_i) + job_i.length()
                    - t_j + t_i > 0.0001 )
                {
                    startVar.add(cbb_p(i,j));
		            startVal.add(1);
		            variables.insert(name_cbb_p_(i,j));                     
                }
		
		        //MINUS VARIABLES
		        //alpha_x(job_j) - alpha_x(job_i)  + t(j) - t(i) >0 => caa_m(i,j)=1
		        if( alpha_x(job_j) - alpha_x(job_i)  + t_j - t_i > 0.0001 )
                {
                    startVar.add(caa_m(i,j));
		            startVal.add(1);
		            variables.insert(name_caa_m_(i,j));                     
                }
		        
		        //beta_x(job_j) + job_j.length() 
		        // -  alpha_x(job_i) + t(j) - t(i) >0 => cab_m(i,j)=1
                if( beta_x(job_j) + job_j.length() -  alpha_x(job_i) + t_j - t_i > 0.0001 )
                {
                    startVar.add(cab_m(i,j));
		            startVal.add(1);
		            variables.insert(name_cab_m_(i,j));                     
                }
                
		        //alpha_x(job_j) - beta_x(job_i) - job_i.length() + t(j) - t(i) > 0
		        // => cba_m(i,j)=1
		        if( alpha_x(job_j) - beta_x(job_i) - job_i.length()+ t_j - t_i > 0.0001 )
                {
                    startVar.add(cba_m(i,j));
		            startVal.add(1);
		            variables.insert(name_cba_m_(i,j));                     
                }
		        
		        //beta_x(job_j) + job_j.length()
		        // - beta_x(job_i) - job_i.length() + t(j) - t(i) > 0
		        // =>  	cbb_m(i,j)=1
                if(beta_x(job_j) + job_j.length() 
                    - beta_x(job_i) - job_i.length()  + t_j - t_i > 0.0001 )
                {
                    startVar.add(cbb_m(i,j));
		            startVal.add(1);
		            variables.insert(name_cbb_m_(i,j));                     
                }

                //CHAIN VARIABLES
                //caa_p(i,j) = caa_m(i,j) =1 => c(i,j)=1		
		        /*if(variables.find(name_caa_p_(i,j))!=variables.end() and    
		           variables.find(name_caa_m_(i,j))!=variables.end())
		        */
		        if(contains(variables,name_caa_p_(i,j)) and 
		           contains(variables,name_caa_m_(i,j)) )
		        {
		             startVar.add(c(i,j));
		             startVal.add(1);
		             //set this variable only once!
		             continue;
		        }
		           
		        //cab_p(i,j) = cab_m(i,j) =1 => c(i,j)=1
		        if(contains(variables,name_cab_p_(i,j)) and 
		           contains(variables,name_cab_m_(i,j)))
		        {
		             startVar.add(c(i,j));
		             startVal.add(1);
		             //set this variable only once!
		             continue;
		        }		
		        
		        //cba_p(i,j) = cba_m(i,j) =1 => c(i,j)=1		
		        if(contains(variables,name_cba_p_(i,j)) and 
		           contains(variables,name_cba_m_(i,j)))
		        {
		             startVar.add(c(i,j));
		             startVal.add(1);
		             //set this variable only once!
		             continue;
		        }
		        
		        //cbb_p(i,j) = cbb_m(i,j) =1 => c(i,j)=1		
                if(contains(variables,name_cbb_p_(i,j)) and 
		           contains(variables,name_cbb_m_(i,j)))
                {
		             startVar.add(c(i,j));
		             startVal.add(1);
		             //set this variable only once!
		             continue;
		        }
            }
        }
        if(debug_){
            cout<<"Variables set to start the MIP:"<<endl;
            for(auto name: variables)
                cout<< name<< " = "<<1<< endl;
            cout << "\n"<< endl;
        }
    }
    
    //add the vector to the system
    cplex_.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
         
}



//------ Here are some GTests for this class---//
#ifdef GTESTS_ENABLED
#include <gtest/gtest.h>
TEST(Three_Index_MIP, Normal) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    independent_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(false);
	mip.set_tightening_cons(false);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ(objective, 221);
    EXPECT_TRUE (i.verify(t));
}

TEST(Three_Index_MIP, SubtourCuts) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    independent_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(true);
	mip.set_tightening_cons(false);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ(objective, 221);
    EXPECT_TRUE(i.verify(t));

}

TEST(Three_Index_MIP, Tightening) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    independent_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(false);
	mip.set_tightening_cons(true);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ (objective, 221);
    EXPECT_TRUE(i.verify(t));
}

TEST(Three_Index_MIP, TighteningAndCuts) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    independent_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(true);
	mip.set_tightening_cons(true);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ(objective, 221);
    EXPECT_TRUE (i.verify(t));
}

#else

#endif