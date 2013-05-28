#include "independent_TSP_MIP.h"

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
 			for(uint v = 1; v <= inst_.num_vehicles();++v){
				string name = name_x_(i,j,v);
				v_[name] = counter_++;
				vars_.add( IloNumVar(env_, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
			}

		}

		
	//tour variable:
	for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
		name = name_k_(j);
		v_[name] = counter_++;
		vars_.add( IloNumVar(env_, 1/*lb*/, inst_.num_vehicles()/*ub*/, type, name.c_str() ) );
	}		
}

										
//constraint construction
void independent_TSP_MIP::build_constraints_(){
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	
	//TODO add a better initialization!
	bigM = 1000;
	//indegree == outdegree for non-depot nodes
	for(uint v=1; v<=K;++v){
		for(uint i = 1; i<=n+K; ++i){
			IloExpr expr(env_);
			//incoming edges
			for(uint j = 1; j<=n+K; ++j){
				if(i>n and j>n) break; //no edges from a depot to another depot
				if(i==j ) continue;
				expr += x(i,j,v);
			}

			//outgoing edges
			for(uint j = 1; j<=n+K; ++j){
				if(i>n and j>n) break; //no edges from a depot to another depot
				if(i==j ) continue;
				expr -= x(j,i,v);
			}

			IloRange constraint(env_, 0, expr, 0,
			("in-outdegree for "+to_string(i)+" vechicle "+to_string(v)).c_str());
			cons_.add(constraint);		
		}
	}
		
	//add IN degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint v = 1; v<=K; ++v){
			for(uint j = 1; j<=n+K; ++j){
				if(i>n and j>n) continue; //no edges from a depot to another depot
	 			if(i==j) continue; //add x_ii only for depot
				expr += x(j,i,v);
			}
		}
		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0, 
							("in-degree for "+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
		
	//add OUT degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint v = 1; v<=K; ++v){
			for(uint j = 1; j<=n+K; ++j){
				if(i>n and j>n) continue; //no edges from a depot to another depot
	 			if(i==j) continue; //add x_ii only for depot
				expr += x(i,j,v);
			}
		}
		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0, 
							("out-degree for "+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
		
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






	//vehicle number for depots is there number
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( k(i) == i-n);
	
		
	//executing vehicle defined by the used edges!
	for(uint i = 1; i<=n; ++i){
		IloExpr expr(env_);
		expr +=  k(i);
		for(uint v=1; v<=K;++v){
			for(uint j = 1; j<=n+K; ++j){
				if(i==j) continue;	//do not respect selfloops
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
	uint K = inst_.num_vehicles();
	//build all k tours
	for( uint i = 1; i<=K; ++i){
		uint current_pos = n+i;//starting at depot i
		do{
			//find next job
			for( uint j = 1; j<=n+K; ++j){
				if(current_pos==j) continue;
				cout<< "trying "<<name_x_(current_pos,j)<< endl;
				if( cplex_.getValue( x(current_pos,j) ) > 0.5 ){
					cout<< name_x_(current_pos,j) <<" = "<<cplex_.getValue( x(current_pos,j) ) << endl;
					double time = cplex_.getValue(t(j));
					if(j<=n){
						cout<<"adding job "<<j<< endl;
						tours.add_job(&inst_[j-1], time,i-1);
					}
					current_pos = j;
					break;
				}	
			}
		}while(n+i!=current_pos);	
	}

	//cplex_.exportModel("mTSP.lp");
	//cplex_.writeSolutions("mTSP.sol"); 
}	

