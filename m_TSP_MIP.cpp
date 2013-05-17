#include "m_TSP_MIP.h"

#define ADD_COLLVAR(NAME) \
    { string name = name_ ## NAME ## _(i,j);\
	v_[name] = counter_++;\
	vars_.add( IloNumVar(env_, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );}\


using namespace std;

void m_TSP_MIP::add_objective_function_(){
	model_.add(IloMinimize(env_,vars_[v_["makespan"]]));
}
		 
//building variables
void m_TSP_MIP::build_variables_(){
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
			string name = name_x_(i,j);
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
void m_TSP_MIP::build_constraints_(){
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	
	//TODO add a better initialization!
	bigM = 1000;
	
	//add OUT degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint j = 1; j<=n+K; ++j){
			if(i>n and j>n) break; //no edges from a depot to another depot
			if(i==j ) continue;
			expr += x(i,j);
		}

		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0,
									 ("out degree for "+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
		
	//add IN degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint j = 1; j<=n+K; ++j){
			if(i>n and j>n) break; //no edges from a depot to another depot
 			if(i==j) continue; //add x_ii only for depot
			expr += x(j,i);
		}
		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0, 
									("in degree for "+to_string(i)).c_str() );
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
		   
			cons_.add( t(j) - 1*t(i) 
					- (job_i.length() + dist_inf(job_i.beta(),job_j.alpha())) * x(i,j) 
					- bigM*x(i,j)  >= -bigM);
		}
	
	//starting time variables for "first" jobs after depots
	for(uint i = n+1; i<=n+K; ++i)
		for(uint j = 1; j<=n; ++j){
			if(i==j) continue;
			const Job& job_j = inst_[j-1];
			cons_.add( t(j)  - dist_inf(inst_.get_depot(i-n-1),job_j.alpha()) * x(i,j) >=0 );
		}	
	
	
	//makespan constraint for every job
	for(uint i=1; i<=n; ++i){
		IloNumVar &makespan = vars_[v_["makespan"]];
		const Job& job = inst_[i-1];
		IloExpr expr(env_);
		expr += 1*makespan;
		expr -= t(i);
		expr -= job.length();
		for(uint j = 1; j<=K; ++j)
			expr -= dist_inf(inst_.get_depot(j-1),job.get_beta()) * x(i,n+j);
			
		IloRange constraint(env_, 0, expr, IloInfinity, ("makespan cons due to vehicle "+to_string(i)).c_str() );
		cons_.add(constraint);	
	}

	//vehicle number for depots is there number
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( k(i) == i-n);
	
	//along edges the number has to stay the same!
	//from depot/job to job
	for(uint i = 1; i<=n+K; ++i)
		for(uint j = 1; j<=n+K; ++j){
			if(i>n and j > n) continue; //no x(i,j) between depots 
			if(i==j) continue;	
			IloExpr expr(env_);
			expr +=  k(i);
			expr -=  k(j);
			expr +=  static_cast<IloInt>(K)* x(i,j);
			
			IloRange constraint(env_, -IloInfinity, expr, K, ("vehicle assignment "+name_x_(i,j)).c_str() );
			cons_.add(constraint);	
		}

}
void m_TSP_MIP::build_collision_constraints_(){
	uint n = inst_.num_jobs();
	auto K = static_cast<IloInt>(inst_.num_vehicles());
	
	//k_j - k_i >= 1 - (1-c_ij)*k <=> k_j - k_i -k*c_ij > = 1 -k 
	for(uint i=1; i<=n; ++i){
		for(uint j=1; j<=n; ++j){
			if(i==j) continue;
			cons_.add( k(j) - k(i) - K*c(i,j) >= 1-K );
		}
	}
	
	//c_ij - cab_p_ij - cab_m_ij >= -1 
	for(uint i=1; i<=n; ++i){
		for(uint j=1; j<=n; ++j){
			if(i==j) continue;
			cons_.add(c(i,j)  - caa_p(i,j) - caa_m(i,j) >= -1);
			cons_.add(c(i,j)  - cab_p(i,j) - cab_m(i,j) >= -1);
			cons_.add(c(i,j)  - cba_p(i,j) - cba_m(i,j) >= -1);
			cons_.add(c(i,j)  - cbb_p(i,j) - cbb_m(i,j) >= -1);
		}
	}
	
	//assigment for all c_ab_p(i,j) variables, see pdf for formulas!
	//int aplus = [](const Job& job){j.get_alpha()[0] -  };
	for(uint i=1; i<=n; ++i){
		for(uint j=1; j<=n; ++j){
			if(i==j) continue;
			
			const Job& job_i = inst_[i-1];
			const Job& job_j = inst_[j-1];
			auto alpha_x = [&](const Job& job)->int{return job.get_alpha()[0];};
			auto beta_x  = [&](const Job& job)->int{return job.get_beta()[0];};
			
			//plus variables
			cons_.add(bigM*caa_p(i,j)  + t(j) - t(i) >= alpha_x(job_j) - alpha_x(job_i) );
			cons_.add(bigM*cab_p(i,j)  + t(j) - t(i) >= beta_x(job_j) - job_j.length() 
													- alpha_x(job_i));
			cons_.add(bigM*cba_p(i,j)  + t(j) - t(i) >= alpha_x(job_j) 
													- beta_x(job_i) + job_i.length());
			cons_.add(bigM*cbb_p(i,j)  + t(j) - t(i) >= beta_x(job_j) -job_j.length()
													- beta_x(job_i) + job_i.length());
		
			//minus variables
			cons_.add(bigM*caa_m(i,j)  - t(j) + t(i) >= alpha_x(job_j) - alpha_x(job_i));
			cons_.add(bigM*cab_m(i,j)  - t(j) + t(i) >= beta_x(job_j) + job_j.length() 
													-  alpha_x(job_i));
			cons_.add(bigM*cba_m(i,j)  - t(j) + t(i) >= alpha_x(job_j) 
													- beta_x(job_i) - job_i.length());
			cons_.add(bigM*cbb_m(i,j)  - t(j) + t(i) >= beta_x(job_j) + job_j.length()
													- beta_x(job_i) - job_i.length());
		}
	}
}
		

void m_TSP_MIP::build_collision_variables_(){
	auto type = IloNumVar::Bool;
	if(LP_relaxation_)
		type = IloNumVar::Float; 

		//c_letter,letter_sign
	for(uint i = 1; i <= inst_.num_jobs();++i){
		for(uint j = 1; j <= inst_.num_jobs();++j){
			//skip variable c_i_i, no need for that
			if(i==j and i<=inst_.num_jobs() )
				continue;
			ADD_COLLVAR(c);
			ADD_COLLVAR(caa_p);
			ADD_COLLVAR(caa_m);
			ADD_COLLVAR(cab_p);
			ADD_COLLVAR(cab_m);
			ADD_COLLVAR(cba_p);
			ADD_COLLVAR(cba_m);
			ADD_COLLVAR(cbb_p);
			ADD_COLLVAR(cbb_m);
		}	
	}
}			
		
//parsing of solution		
void m_TSP_MIP::parse_solution_(Tours &tours){
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

