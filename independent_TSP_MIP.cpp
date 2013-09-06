#include "independent_TSP_MIP.h"
#include "DisjointSet.h"
#include <set>
#include <thread>
using namespace std;

void independent_TSP_MIP::add_objective_function_(){
	model_.add(IloMinimize(env_,vars_[v_["makespan"]]));
}
		 
//building variables
void independent_TSP_MIP::build_variables_(){
	auto type = IloNumVar::Bool;
	if(LP_relaxation_)
		type = IloNumVar::Float; 


	//additional depot and 
	string name = "makespan";
	v_[name] = counter_++;
	vars_.add( IloNumVar(env_, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );

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
		vars_.add( IloNumVar(env_, 1/*lb*/, inst_.num_vehicles()/*ub*/,IloNumVar::Int, name.c_str() ) );
	}	
	
		
}

										
//constraint construction
void independent_TSP_MIP::build_constraints_(){
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	
	//TODO add a better initialization!
	bigM = 1000;

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
			("in-outdegree for "+to_string(i)+" vehicle "+to_string(v)).c_str());
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
							("in-degree for "+to_string(i)).c_str() );
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
							("out-degree for "+to_string(i)).c_str() );
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
				
				cons_.add( t(j) - 1*t(i)- bigM*x(i,j,v)  
							>=  job_i.length() + dist_inf(job_i.beta(),job_j.alpha())
								 -bigM);
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
		IloNumVar &makespan = vars_[v_["makespan"]];
		const Job& job = inst_[i-1];
		IloExpr expr(env_);
		expr += 1*makespan;
		expr -= t(i);
		expr -= job.length();
		for(uint v = 1; v<=K; ++v)
			expr -= dist_inf(inst_.get_depot(v-1),job.get_beta()) * x(i,n+v,v);
			
		IloRange constraint(env_, 0, expr, IloInfinity, ("makespan cons due to vehicle "+to_string(i)).c_str() );
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
				("vehicle assignment "+to_string(i)).c_str() );
		cons_.add(constraint);
	}	

}

	
//parsing of solution		
void independent_TSP_MIP::parse_solution_(Tours &tours){

	//TODO: remove this line after debuging
	print_LP_solution_();

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
	auto arc_name = [&] (int i, int j, int k){ return mip->name_x_(i,j,k);};
		
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
	
		build_variables_();
		
		if( 1 != inst_.num_vehicles() and  collision_avoidance_ )
			build_collision_variables_();
		
		model_.add(vars_);
			
		//objective function: minimize makespan
		add_objective_function_();
		
		build_constraints_();
	
		if( 1!= inst_.num_vehicles() and  collision_avoidance_)
			build_collision_constraints_();  

		model_.add(cons_);

        //add MIP start
        add_MIP_start();

		//run cplex and solve the model!
		cplex_.extract(model_);
		
		//print small models 
		if( inst_.num_jobs()<=6)
			cout<<"MIP Model: \n"<<model_<<endl;
		
		//add callback to create cuts on the fly
		uint threads = thread::hardware_concurrency();
		cplex_.use( SubtourCutsCallback(env_, this) );
		cplex_.setParam(IloCplex::IntParam::Threads	,threads);
		cout<<"Detected "<<threads<<" cores" <<endl;
			
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


void independent_TSP_MIP::add_MIP_start(){
    if(start.empty()) return;
    auto schedule = start.get_schedule();

    uint n = inst_.num_jobs();
 	uint K = inst_.num_vehicles();
        
    IloNumVarArray startVar(env_);
    IloNumArray startVal(env_);


    //makespan,	a variable for the makespan
    startVar.add(vars_[v_["makespan"]]);
    startVal.add(inst_.makespan(start));
    
    //t_j, 		starting times for all jobs and all depots
    for(int i=1; i<=n; ++i){
        startVar.add(t(i));
        //starting time shopuld be an integer! or at least close to it
        assert( fabs(get<1>(schedule[i])- (int)(get<1>(schedule[i])))<.01 ); 
        
        startVal.add( (int)(get<1>(schedule[i])) );
    }
    
    //non-used variables are 0?
    //TODO: my guess: not defined vars are zero....test this assumption
    //x_i,j,k for used edges between jobs
    for(int v=0; v< K; ++v)
        for(int i=0; i<start[v].size()-1;++i){
            
            int job1 = std::get<0>(start[v][i])->num();
            int job2 = std::get<0>(start[v][i+1])->num();
                           
            startVar.add(x(job1,job2,v+1));
            startVal.add( 1 );
                    
        }
        
      //edges from depot to first job AND from last to depot
            
       
    
    
    //k_j,  \in (0,...,k-1),tour variable, which assigns a job to a vehicle	
    for(int i=1; i<=n; ++i){
        startVar.add(k(i));
        startVal.add(get<2>(schedule[i]));
    }
   
    
    cplex_.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
    
    /*
    Code Example from IBM Ilog: 
    
   Use the method IloCplex::addMIPStart to add a MIP start to your model. This method is not incremental. In other words, successive calls of this method do not add more values to an existing MIP start. Instead, successive calls of the method override any existing MIP start. That is, each call of this method creates a new MIP start.
   
Furthermore, this method works only on one-dimensional arrays. If you want to create a MIP start from a multidimensional array, you first must flatten the multidimensional array by copying the variables into a one-dimensional array before you call this method. Here is a sample, assuming a matrix of dimensions m by n, with the starting value x[i][j] in start[i][j], that you can adapt to your own application. 

    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j) {
            startVar.add(x[i][j]);
            startVal.add(start[i][j]);
        }
     cplex.addMIPStart(startVar, startVal);
     startVal.end();
     startVar.end();
    */
    
         
}

