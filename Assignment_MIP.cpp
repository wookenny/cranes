#include "Assigment_MIP.h"

using namespace std;

void Assigment_MIP::add_objective_function_(){
	model_.add(IloMinimize(env_,vars_[v_["makespan"]]));
}
		 
//building variables
void Assigment_MIP::build_variables_(){
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
	

}

										
//constraint construction
void Assigment_MIP::build_constraints_(){
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	
	//TODO add a better initialization!
	bigM = 1000;
	
	
		
	//starting time variables for depots
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( t(i) == 0);	
	
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
}


void Assigment_MIP::build_collision_constraints_(){
	//TODO build the constraints!
}
		

void Assigment_MIP::build_collision_variables_(){
	//TODO: build these variables
}			
		
//parsing of solution		
void Assigment_MIP::parse_solution_(Tours &tours){
	//TODO: remove this line after debuging
	print_LP_solution_();

	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	//build all k tours
	
}	

