#include "generalizedVRP_MIP.h"

#include "Job.h"
#include "Tours.h"

//makro to add collision variables
#define ADD_COLLVAR(NAME) \
    { string name = name_ ## NAME ## _(i,j);\
	v_[name] = counter_++;\
	vars_.add( IloNumVar(env_, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );}\
	
	


using namespace std;

std::pair<Tours,double> generalizedVRP_MIP::solve(){
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
			
		//objective function: minimize makespan
		if(fixed_makespan_ < 0)
		    add_objective_function_();    
		
		build_constraints_();
	
		if( 1!= inst_.num_vehicles() and  collision_avoidance_)
			build_collision_constraints_();  

		model_.add(cons_);  
        
		//run cplex and solve the model!
		cplex_.extract(model_);
			
		//print small models 
		if(not silent_ and inst_.num_jobs()<=6)
			cout<<"MIP Model: \n"<<model_<<endl;
		
		bool solved = cplex_.solve();
		if(debug_) cout<< "Solving MIP was successful: "<<boolalpha<<solved<<endl;
		if(debug_) cout<< "MIP status = "<<cplex_.getStatus()<<endl;
		if ( !solved ) {
			env_.end();
			//return empty tours if MIP was unsolvable!
			return pair<Tours,double>{Tours(inst_.num_vehicles()),
												found_objective_};
		}else{
			//parse solution
			if(LP_relaxation_ and inst_.num_jobs()<10){
				print_LP_solution_();
			}
			found_objective_ = cplex_.getObjValue();	

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
	return pair<Tours,double>{tours,found_objective_};
}

void generalizedVRP_MIP::print_LP_solution_() const{
	cout<<"Values of the solution:" <<endl;
	for(auto& kv : v_){ 
		assert( kv.first == vars_[kv.second].getName());
		double value = cplex_.getValue(vars_[kv.second]);
		if(value > 0.01 or value < -0.01){
			cout<< kv.first <<" = "<< value <<endl;			
		}	
	}
	cout<<endl;
}


void generalizedVRP_MIP::set_start_solution(const Tours&){
    std::cout<<"WARNING: Set MIP start feature not implemented"<<std::endl;
    assert(false);
}

void generalizedVRP_MIP::build_collision_variables_(){
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



void generalizedVRP_MIP::build_collision_constraints_(){
	
	if(bigM<=0)
	    bigM = inst_.get_upper_bound();
    if(fixed_makespan_>0)
        bigM = fixed_makespan_;

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
	for(uint i=1; i<=n; ++i){
		for(uint j=1; j<=n; ++j){
			if(i==j) continue;
			
			const Job& job_i = inst_[i-1];
			const Job& job_j = inst_[j-1];
			auto alpha_x = [&](const Job& job)->int{return job.get_alpha()[0];};
			auto beta_x  = [&](const Job& job)->int{return job.get_beta()[0];};
			
			//plus variables
			//TODO: add rightHandSide to bigM
			cons_.add( (bigM + alpha_x(job_j) - alpha_x(job_i))*caa_p(i,j) 
						 + t(j) - t(i) 
						>= alpha_x(job_j) - alpha_x(job_i) );
			cons_.add( (bigM + beta_x(job_j) - job_j.length()- alpha_x(job_i)) 
						* cab_p(i,j) + t(j) - t(i) 
							 >= beta_x(job_j) - job_j.length()- alpha_x(job_i));
			cons_.add( (bigM+  alpha_x(job_j)- beta_x(job_i) + job_i.length()) 
						* cba_p(i,j)  + t(j) - t(i) 
						>= alpha_x(job_j)- beta_x(job_i) + job_i.length());
			cons_.add((bigM +beta_x(job_j) -job_j.length()
							- beta_x(job_i) + job_i.length())*cbb_p(i,j)
							+ t(j) - t(i) 
						>= beta_x(job_j) -job_j.length()
							- beta_x(job_i) + job_i.length());
		
			//minus variables
			cons_.add( (bigM+alpha_x(job_j) - alpha_x(job_i))
						* caa_m(i,j)  - t(j) + t(i) 
					 	>= alpha_x(job_j) - alpha_x(job_i));
			cons_.add( (bigM+beta_x(job_j) + job_j.length()-alpha_x(job_i))
						*cab_m(i,j)  - t(j) + t(i) 
						>= beta_x(job_j) + job_j.length()-alpha_x(job_i));
			cons_.add( (bigM+ alpha_x(job_j) - beta_x(job_i) - job_i.length()) 
						* cba_m(i,j)  - t(j) + t(i) 
						>= alpha_x(job_j) - beta_x(job_i) - job_i.length());
			cons_.add( (bigM + beta_x(job_j) + job_j.length()-beta_x(job_i) - job_i.length())
						*cbb_m(i,j)  - t(j) + t(i) 
						>= beta_x(job_j) + job_j.length()-beta_x(job_i) - job_i.length());
		}
	}
	
}





