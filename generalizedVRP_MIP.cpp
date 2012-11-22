#include "generalizedVRP_MIP.h"

#include "Job.h"
#include "Tours.h"



using namespace std;

Tours generalizedVRP_MIP::solve(){
	Tours tours(inst_.num_vehicles());

	//build the MIP model and solve it!
	try {
		model_ 	= IloModel(env_);
		cplex_ 	= IloCplex(env_);
		vars_ 	= IloNumVarArray(env_);		
		
		build_variables_();
		
		if( 1 != inst_.num_vehicles() and  collision_avoidance_)
			build_collision_variables_();
		
		model_.add(vars_);
			
		//objective function: minimize makespan
		add_objective_function_();

		
		build_constraints_();
	
		if( 1!= inst_.num_vehicles() and  collision_avoidance_)
			build_collision_constraints_();  

		//run cplex and solve the model!
		cplex_.extract(model_);
		
		//print small models 
		if( inst_.num_jobs()<=6)
			cout<<"MIP Model: \n"<<model_<<endl;
		
		bool solved = cplex_.solve();
		if ( !solved ) {
			env_.end();
			//return empty tours if MIP was unsolvable!
			return Tours(inst_.num_vehicles());
		}else{
			//parse solution
			//return empty tour id only an LP relaxation was solved
			if(LP_relaxation_){
				print_LP_solution_();
				return tours;
			}	
			parse_solution_(tours);
		}
		
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

void generalizedVRP_MIP::print_LP_solution_() const{
	for(auto& kv : v_) 
		cout<< kv.first <<" = "<< cplex_.getValue(vars_[kv.second]) <<endl;	
}
