#include <ilcplex/ilocplex.h>
#include "iostream.h"


int main(){

	IloEnv env;
	try {
		IloModel model(env);
		IloNumVarArray vars(env);
		
		
		vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, IloNumVar::Float/*type*/, name.c_str() ) );
		vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, IloNumVar::Float/*type*/, name.c_str() ) );
		
		model.add(IloMaximize(env, 0));
		
		model.add(  vars[0]  - vars[1]  >= 2);
		
		IloCplex cplex(env);
		cplex.solve();
	
	}catch (IloException& e) {
		std::cerr << "Concert exception caught: " << e << std::endl;
	}
	catch (...) {
		std::cerr << "Unknown exception caught" << std::endl;
	}
	env.end();	
	
}
