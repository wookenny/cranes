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

	//indegree == outdegree for non-depot nodes
	for(uint v=1; v<=K;++v){
		for(uint i = 1; i<=n+K; ++i){
			if(i>n and v!=i-n) //constraint with veh. i for depot j
					continue;
		
			IloExpr expr(env_);
			//outgoing edges(from i)
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and i>n) continue; //no interdepot constraint
				if(i==j ) continue;//no selfedge
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;

				expr += x(i,j,v);
			}
			//incoming edges(from i)
			for(uint j = 1; j<=n+K; ++j){
				if(i==j ) continue;
				if(j>n and i>n) continue; //no interdepot constraint
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;

				expr -= x(j,i,v);
			}

			IloRange constraint(env_, 0, expr, 0,
			("in-outdegree for "+to_string(i)+" vehicle "+to_string(v)).c_str());
			cons_.add(constraint);		
		}
	}
			
	//add IN degree for vertices
	for(uint i = 1; i<=n; ++i){
		IloExpr expr(env_);
		for(uint v = 1; v<=K; ++v){
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;
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
	for(uint i = 1; i<=n; ++i){
		IloExpr expr(env_);
		for(uint v = 1; v<=K; ++v){
			for(uint j = 1; j<=n+K; ++j){
				if(j>n and v!=j-n) //no edges from depot with wrong vehicle 
					continue;
	 			if(i==j) continue; //add x_ii only for depot
				expr += x(i,j,v);
			}
		}
		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0, 
							("out-degree for "+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
	
	//special constraint for the depot:
	//sum of out/in <=1 
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

