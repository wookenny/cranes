#pragma once

#include <unordered_map>
#include <string>

#include "Instance.h"

class Tour;
class Job;
class IloEnv;
class IloNumVarArray;
class IloModel;
class IloCplex;

class MIP{

	private:
		const Instance& _inst;
		std::unordered_map<std::string,int> v;
		int M;
		int counter;
	public:
		MIP()=delete;
		MIP(const Instance& i):_inst(i),v(),M(i.get_upper_bound()),counter(0){}
		
		Tours solve(bool collision_free = false, bool LP_relax = false, bool debug = false);

	private:
		//variable construction
		void _build_variables(IloEnv &env, IloNumVarArray &vars,bool LP_relax = false); 
		void _build_collision_variables(IloEnv &env, IloNumVarArray &vars,bool LP_relax = false);	
		void _build_variables_single_vehicle(IloEnv &env, IloNumVarArray &vars,bool LP_relax = false);
										
		//constraint construction
		void _build_constraints(IloEnv &env, IloNumVarArray &vars,IloModel &model);
		void _build_collision_constraints(IloEnv &env, IloNumVarArray &vars,
											IloModel &model);  
		void _build_constraints_single_vehicle(IloEnv &env, 
										IloNumVarArray &vars,IloModel &model);
		
		//parsing of solution		
		void _parse_solution(IloCplex &cplex, Tours &tours,
									IloNumVarArray &vars,bool debug=false) ;
		void _parse_solution_single_vehicle(IloCplex &cplex, Tours &tours,
									IloNumVarArray &vars,bool debug=false) ;
									
		void _print_LP_solution(const IloCplex &cplex,const IloNumVarArray &vars) const;							
};
