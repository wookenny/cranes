#include "independent_TSP_MIP.h"
#include "DisjointSet.h"
#include <set>
#include <thread>
#include<algorithm>
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
				vars_.add( IloNumVar(env_, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
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
		vars_.add( IloNumVar(env_, 1/*lb*/, inst_.num_vehicles()/*ub*/,var_type, name.c_str() ) );
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
	
	/*
	//special constraint for the depot:
	//sum of out/in <=1  for every sngle depot
	for(uint i = n+1; i<=n+K; ++i){
		IloExpr expr_out(env_);
		IloExpr expr_in(env_);
		int v = i-n;
		for(uint j = 1; j<=n; ++j){
	 		if(i==j) continue; //add x_ii only for depot
			expr_out += x(i,j,v);
			expr_in  += x(j,i,v);
		}
		IloRange out_constraint(env_, 0, expr_out, 1, 
							("out-degree for "+to_string(i)).c_str() );
		IloRange in_constraint(env_, 0, expr_in, 1, 
							("in-degree for "+to_string(i)).c_str() );
		cons_.add(out_constraint);	
		cons_.add(in_constraint);		
	}
   */


	//starting time variables for depots
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( t(i) == 0);	
	
	
	//starting time variables for regular jobs
	for(uint i = 1; i<=n; ++i)
		for(uint j = 1; j<=n; ++j){
			if(i==j) continue;
			const Job& job_i = inst_[i-1];
			const Job& job_j = inst_[j-1];
			for(uint v =1; v<=K;++v){
				auto tmpBigM = bigM+ job_i.length() + dist_inf(job_i.beta(),job_j.alpha());  
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
		            l += dist_inf(j1.beta(),inst_.get_depot(v-1));	            
		            expr -=  l * x(i,n+v,v);
		            //from depot
		            l = dist_inf(inst_.get_depot(v-1),j1.alpha());
		            expr -=  l * x(n+v,i,v);
		    }
			    
			IloRange constraint(env_, 0, expr, IloInfinity,
			("length_bound_for_tour_"+to_string(v)).c_str());
			cons_.add(constraint);		
	}
	
}

	
//parsing of solution		
void independent_TSP_MIP::parse_solution_(Tours &tours){


	uint n = inst_.num_jobs();

	//build all k tours
	for( uint j = 1; j<=n; ++j){
		int v = cplex_.getValue(k(j));
		double time = cplex_.getValue(t(j));
		tours.add_job(&inst_[j-1], time, v-1);
	}
	tours.sort_jobs();

	//cplex_.exportModel("mTSP.lp");
	//cplex_.writeSolutions("mTSP.sol"); 
}	


/**
 Own solve()-method to enable an own cut-callback
**/

//Cut callback 
//checks for fractional subtours, which means: ?????
ILOUSERCUTCALLBACK1(SubtourCutsCallback, independent_TSP_MIP*, mip )
{       
   // Skip the separation if not at the end of the cut loop
   if( !isAfterCutLoop() )
      return;
	            
 	uint n = mip->inst_.num_jobs();
 	uint K = mip->inst_.num_vehicles();
    
    //find elements that are reachable from a depot
    //ask vor every valie of an edge
    DisjointSet sets( n+K );
    //all depots are interconnected! connect d_i with d_i+1
    for(uint j = n+1; j<= n+K-1; ++j)
    	sets.unionSets(j-1,j+1-1);
   
    
	//cplex style to get the model
	IloEnv env = getEnv();
	IloInt numNodes =  mip->vars_.getSize();
	IloNumArray xSol(env, numNodes);
	getValues(xSol,  mip->vars_);

	auto arc = [&] (int i, int j, int k){ return xSol[mip->v_[mip->name_x_(i,j,k)]];};
	//auto arc_name = [&] (int i, int j, int k){ return mip->name_x_(i,j,k);};
		
	for(uint i=1; i<= n; ++i){
		//reaching all real jobs
		
		for(uint j = 1; j<= n; ++j){
			for(uint v = 1; v<=K; ++v){
				if(i==j) continue;
				if( arc(i,j,v) > 0.0001)
					sets.unionSets(i-1,j-1);
	   			}
	   	}
		
		//reaching all deopts 
		for(uint j = n+1; j<= n+K; ++j){
			if( arc(i,j,j-n) > 0.0001)
				sets.unionSets(i-1,j-1);
   		}
	}
	
	//starting at depot:
	for(uint i=n+1; i<= n+K; ++i){
		//reaching real jobs
		for(uint j = 1; j<= n; ++j){
			if( arc(i,j,i-n)> 0.0001)
				sets.unionSets(i-1,j-1);	
		}			
	}

  
   //cout<<"\n\nfound: "<<sets.size()<<" components "<<endl;
   if(sets.size()!=1){
   			//print all values, 
			std::set<uint> component1;
			component1.insert(1);
			uint cmp = sets.findSet(0);			
			for(uint j = 1; j<= n+K; ++j){
				uint current_Set = sets.findSet(j-1);
				if( current_Set == cmp )
					component1.insert(j);
			}
			
			//build constraint	
		 	IloExpr cutLhs1(env);
	 		IloExpr cutLhs2(env);

		 	//find other "side" of the cut
		 	set<uint> component2;
		 	for(uint i=1; i<=n+K; ++i)
		 		if( component1.find(i)==component1.end())
		 			component2.insert(i);	
		 				 
		 	for(auto i: component1)
				for(auto j: component2){
					if(i>n and j>n) continue; //no interdepot edges
					if(i>n){//only i is a depot
						cutLhs1 += 1*mip->x(i,j,i-n);//outgoing edge
						cutLhs2 += 1*mip->x(j,i,i-n);//incoming edge
					}else if(j>n){//only j is a depot
						cutLhs1 += 1*mip->x(i,j,j-n);//outgoing edge
						cutLhs2 += 1*mip->x(j,i,j-n);//incoming edge
					}else{
					//i and j are no depots
						for(uint v=1; v<=K; ++v){
							cutLhs1 += 1*mip->x(i,j,v);//outgoing edge
							cutLhs2 += 1*mip->x(j,i,v);//incoming edge
						}
					}							
				}
		 	
		 	//add both cuts to the model
   			IloNum cutRhs = 1;			 
   			
   						
   		    add(cutLhs1 >= cutRhs).end();
   		    add(cutLhs2 >= cutRhs).end();
   		  //  cout<<"added 2 cuts"<<endl;
   		    //cout<< cutLhs1 <<" >= "<<cutRhs <<endl;
   			cutLhs1.end(); cutLhs2.end();

		//Debug Info: Solution printed!
		/*
			//print a bad set
			//print variables for the solution!

			for(uint i=1; i<= n; ++i){
			//reaching all real jobs
		
			for(uint j = 1; j<= n; ++j){
				for(uint v = 1; v<=K; ++v){
					if(i==j) continue;
					if( arc(i,j,v) > 0.0001)
						cout<<arc_name(i,j,v) <<" "<<arc(i,j,v) <<endl;
		   			}
		   	}
		
			//reaching all deopts 
			for(uint j = n+1; j<= n+K; ++j){
				if( arc(i,j,j-n) > 0.0001)
					cout<<arc_name(i,j,j-n) <<" "<<arc(i,j,j-n) <<endl;
	   		}
		}
	
		//starting at depot:
		for(uint i=n+1; i<= n+K; ++i){
			//reaching real jobs
			for(uint j = 1; j<= n; ++j){
				if( arc(i,j,i-n)> 0.0001)
					cout<<arc_name(i,j,i-n) <<" "<<arc(i,j,i-n) <<endl;	
			}			
		}
	*/	
	//assert(false);	
	}
	
   
   //free all cplex stuff
   xSol.end();
   return;
}

Tours independent_TSP_MIP::solve(){
	Tours tours(inst_.num_vehicles());

	//build the MIP model and solve it!
	try {
	
		if(silent_)
			cplex_.setParam(IloCplex::MIPDisplay, 0);

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
		if ( !solved ) {
			env_.end();
			//return empty tours if MIP was unsolvable!
			return Tours(inst_.num_vehicles());
		}else{
			//parse solution
			//return empty tour id only an LP relaxation was solved
			if(LP_relaxation_ and inst_.num_jobs()<10){
				print_LP_solution_();
				return tours;
			}	
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
	env_.end();
	
	return tours;
}


void independent_TSP_MIP::add_MIP_start_(){
    if(LP_relaxation_ or start.empty() ) return;
    if(fixed_makespan_>0) return; //not sound to set a start in this setting
    
    auto schedule = start.get_schedule();

    uint n = inst_.num_jobs();
 	uint K = inst_.num_vehicles();
        
    IloNumVarArray startVar(env_);
    IloNumArray startVal(env_);

    //makespan,	a variable for the makespan
    startVar.add(vars_[v_["makespan"]]);
    startVal.add(inst_.makespan(start));
      
    //t_j, 		starting times for all jobs and ll depots
    for(uint i=1; i<=n; ++i){
        startVar.add(t(i));
        //starting time shopuld be an integer! or at least close to it
        assert( fabs(get<1>(schedule[i])- (int)(get<1>(schedule[i])))<.01 );       
        startVal.add( (int)(get<1>(schedule[i])) );
    }

    //x_i,j,k for used edges between jobs
    for(uint v=0; v< K; ++v)
        for(uint i=0; i+1 < start[v].size(); ++i){
            int job1 = std::get<0>(start[v][i])->num();
            int job2 = std::get<0>(start[v][i+1])->num();
                           
            startVar.add(x(job1,job2,v+1));
            startVal.add( 1 );
        }

    //edges from depot to first job AND from last to depot
    for(uint v=0; v< K; ++v){
        if(start[v].empty()) continue;
    
         int first_job = std::get<0>(start[v].front())->num();
         int last_job = std::get<0>(start[v].back())->num();
         
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

