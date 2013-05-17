#include "generalizedVRP_MIP.h"

#include "Job.h"
#include "Tours.h"



using namespace std;

Tours generalizedVRP_MIP::solve(){
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

		//run cplex and solve the model!
		cplex_.extract(model_);
		
		//print small models 
		if( inst_.num_jobs()<=6)
			cout<<"MIP Model: \n"<<model_<<endl;
		
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
			if(LP_relaxation_){
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

void generalizedVRP_MIP::print_LP_solution_() const{
	cout<<"Values of the solution:" <<endl;
	for(auto& kv : v_){ 
		assert( kv.first == vars_[kv.second].getName());
		double value = cplex_.getValue(vars_[kv.second]);
		if(value > 0.01 or value < -0.01)
			cout<< kv.first <<" = "<< value <<endl;	
	}
	cout<<endl;
}
