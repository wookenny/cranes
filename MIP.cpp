#include "MIP.h"

#include <ilcplex/ilocplex.h>
#include <iostream>

#include "Instance.h"

ILOSTLBEGIN 

//functions to refer the different variables easily
//starting time variable for job i
std::string t(int i){
	return  "t_"+std::to_string(i);
}
std::string t(const Job& i){return t(i.num());}

//x_i_k =1 => job i is processed by vehicle k
std::string x(int i, int k){
	return  "x_"+std::to_string(i)+"_"+std::to_string(k);
}
std::string x(const Job& i,int k){return x(i.num(),k);}

//y_i_j =1 => job i starts earlier than job j
std::string y(int i, int j){
	assert(i!=j);
	return  "y_"+std::to_string(i)+"_"+std::to_string(j);
}
std::string y(const Job& i,const Job& j){return y(i.num(),j.num());}

//z_i_j =1 => job i is handled by the same vehicle as job j
std::string z(int i, int j){
	assert(i!=j);
	if(i<j)
		return  "z_"+std::to_string(i)+"_"+std::to_string(j);
	else
		return  "z_"+std::to_string(j)+"_"+std::to_string(i);
}
std::string z(const Job& i,const Job& j){return z(i.num(),j.num());}

//l_i_j =1 => job i is on vehicle left of job j
std::string l(int i, int j){
	assert(i!=j);
	return  "l_"+std::to_string(i)+"_"+std::to_string(j);
}
std::string l(const Job& i,const Job& j){return l(i.num(),j.num());}

//r_i_j =1 => job i is on vehicle right of job j
std::string r(int i, int j){
	assert(i!=j);
	return  "r_"+std::to_string(i)+"_"+std::to_string(j);
}
std::string r(const Job& i,const Job& j){return r(i.num(),j.num());}

Tours MIP::solve(){

	Tours tours(_inst.num_vehicles());

	//build the MIP model and solve it!
	IloEnv env;
	try {
		IloModel model(env);
		IloCplex cplex(env);
		IloNumVarArray vars(env);
				
		/***********************************************************************
		 				Constructing all necessary variables
		***********************************************************************/
		
		if(_inst.num_vehicles()==1)
			_build_variables_single_vehicle(env, vars, model);
		else
			_build_variables(env, vars, model);
			
		//objective function: minimize makespan
		model.add(IloMinimize(env,vars[v["makespan"]]));	
		
		if(_inst.num_vehicles()==1)
			_build_constraints_single_vehicle(env,vars,model);
		else		
			_build_constraints(env,vars,model);
	
		//run cplex anf solve the model!
		
		cplex.extract(model);
		std::cout<<model<<std::endl;
		bool solved = cplex.solve();
		if ( !solved ) {
			env.end();
			return Tours(_inst.num_vehicles());
		}else{
			//parse solution
			if(_inst.num_vehicles()==1)
				_parse_solution_single_vehicle(cplex, tours,vars);
			else{
				_parse_solution(cplex, tours,vars);
				}
		}
		
		

		
	}catch (IloException& e) {
		std::cerr << "Concert exception caught: " << e << std::endl;
	}
	catch (...) {
		std::cerr << "WARNING: Unknown exception caught" << std::endl;
	}
	env.end();

	return tours;
}

void MIP::_parse_solution(IloCplex &cplex, Tours &tours,IloNumVarArray &vars, bool debug){
	//find assiments and starting times
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[v[t(j)]]);
		for(unsigned int k=1;k<=_inst.num_vehicles(); ++k){
			if(cplex.getValue(vars[v[x(j,k)]])>=0.5){
				tours.add_job(&j, tj, k-1);
			}
		}	
	}

	//if nothing to debug, we can finish here
	if(not debug) return;
	
	//print additional variables for debug informations
	std::cout<< "-------------DEBUG HINTS-------------"<<std::endl;
	std::cout<< "starting times and assignment variables:"<<std::endl;
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[v[t(j)]]);
		std::cout<<"t_"<<j.num()<<" = "<<tj<<"\t";	
		for(unsigned int k=1;k<=_inst.num_vehicles(); ++k){
			if(cplex.getValue(vars[v[x(j,k)]])>=0.1){
				std::cout<< "veh. "<<k<<std::endl;
			}
		}	
	}
	
	std::cout<< "\nprecedence variables:"<<std::endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i==j) continue;
			if(cplex.getValue(vars[v[y(i,j)]])>0.1)
				std::cout<< y(i,j) << " = "<<	cplex.getValue(vars[v[y(i,j)]])<<std::endl;
		}
	}
	
	std::cout<< "\nsame vehicle variables:"<<std::endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i.num()>=j.num()) continue;
			if(cplex.getValue(vars[v[z(i,j)]])>0.1)
				std::cout<< z(i,j) << " = "<<	cplex.getValue(vars[v[z(i,j)]])<<std::endl;
		}
	}
	
	std::cout<< "\nL/R variables:"<<std::endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i==j) continue;
			if(cplex.getValue(vars[v[l(i,j)]])>0.1)
				std::cout<< l(i,j) << " = "<<	cplex.getValue(vars[v[l(i,j)]])<<std::endl;
			if(cplex.getValue(vars[v[r(i,j)]])>0.1)
				std::cout<< r(i,j) << " = "<<	cplex.getValue(vars[v[r(i,j)]])<<std::endl;	
		}
	}
	std::cout<< "-------------END OF DEBUG HINTS-------------"<<std::endl;
	
}

/**Adding all necessary variables to the var array and an additional redirect in the map to use the nicer name of the
variables for direct access.
**/
void MIP::_build_variables(IloEnv &env, IloNumVarArray &vars,IloModel &model, bool LP_relax){
	auto type = IloNumVar::Bool;
	if(LP_relax)
		type = IloNumVar::Float; 
	
	int counter = 0;
	
	//single makespan variable
	std::string name = "makespan";
	v[name] = counter++;
	vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
	
	for(const Job& i: _inst){
		//t: time variable
		std::string name = t(i);
		v[name] = counter++;
		vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
		
		//x: assignment variable(x_i_j=1 => job i done by vehicle j)
		for(unsigned int k=1; k<=_inst.num_vehicles(); ++k){
			std::string name = x(i,k);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, type, name.c_str() ) );
		}

		
		//y: precedence variable(y_i_j)
		for(const Job& j: _inst){
			//skip variable y_i_i
			if(i.num()==j.num()) 
				continue;
			std::string name = y(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, type, name.c_str() ) );
		}	
		
		//z: same vehicle variable(z_i_j)		
		for(const Job& j: _inst){					
			//only variables z_i_j for i < j
			if(i.num()>=j.num()) 
				continue;
			std::string name = z(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, type, name.c_str() ) );
		}
		
		//r: one a vehicle more to the right(r_i_j)
		for(const Job& j: _inst){
			//skip variable r_i_i
			if(i.num()==j.num()) 
				continue;
			std::string name = r(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, type, name.c_str() ) );
		}
		
		//l: one a vehicle more to the left(l_i_j)
		for(const Job& j: _inst){
			//skip variable l_i_i
			if(i.num()==j.num()) 
				continue;
			std::string name = l(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, type, name.c_str() ) );
		}
				
	}
	model.add(vars);
} 





void MIP::_build_constraints(IloEnv &env, IloNumVarArray &vars,IloModel &model){

		//makespan at least as late as last return of all vehivles
		for(const Job& j: _inst){
			IloExpr expr(env);
			expr += 1*vars[v["makespan"]];
			expr -= 1*vars[v[t(j)]];
			for(unsigned int k=1; k<=_inst.num_vehicles(); ++k){
				//subtract dist from job to depot, for each depot
				expr -= dist_inf<int>( j.get_beta(),_inst.get_depot(k-1) ) * vars[v[x(j,k)]];    
			}
			//IloRange(const IloEnv env, IloNum lhs, const IloNumExprArg expr, IloNum rhs, const char * name)
			model.add(IloRange(env, j.length(), expr, IloInfinity,("makespan setting for "+t(j)).c_str() ));
		}
		
		//assignment of job to vehicle
		for(const Job& j: _inst){
			IloExpr expr(env);
			expr += 1*vars[v[x(j,1)]];
			for(unsigned int k=2; k<=_inst.num_vehicles(); ++k){
				//subtract dist from job to depot, for each depot
				expr += 1*vars[v[x(j,k)]];    
			}
			//IloRange(const IloEnv env, IloNum lhs, const IloNumExprArg expr, IloNum rhs, const char * name)
			model.add(IloRange(env, 1, expr, 1, ("assignemt to vehicle for j_"+std::to_string(j.num())).c_str() ));
		}
		
		//precedence variable setting 1
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i.num()==j.num()) continue;
				model.add( M*vars[v[y(i,j)]] + vars[v[t(i)]]  -  vars[v[t(j)]]  >= 0 );
			}
		}
		
		//precedence variable setting 2
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i.num()==j.num()) continue;
				model.add( vars[v[y(i,j)]]  + vars[v[y(j,i)]] == 1 );
			}
		}
		
		//each job reachable from its depotposition in time
		for(const Job& j: _inst){
			IloExpr expr(env);
			expr += 1*vars[v[t(j)]];
			
			for(unsigned int k=1; k<=_inst.num_vehicles(); ++k){
				expr -=  dist_inf<int>( _inst.get_depot(k-1),j.get_alpha() ) * vars[v[x(j,k)]]; 
			}
			model.add(IloRange(env, 0, expr, IloInfinity, ("starttime reachable from depot for j_"+std::to_string(j.num())).c_str() ));
		}
		
		//in a right/left tour constr. no.1
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				//build constraint
				IloExpr expr_l(env), expr_r(env);	
				expr_l += IloInt(_inst.num_vehicles()) * vars[v[l(i,j)]];
				expr_r += IloInt(_inst.num_vehicles()) * vars[v[r(i,j)]];
				for(unsigned int k=1; k<=_inst.num_vehicles(); ++k){
					expr_l += IloInt(k)*vars[v[x(i,k)]] - IloInt(k)*vars[v[x(j,k)]];
					expr_r -= IloInt(k)*vars[v[x(i,k)]] - IloInt(k)*vars[v[x(j,k)]];
				}	
				
				//add both contraints
				model.add(IloRange(env, 0, expr_l, 
						IloInfinity, ("left no.1 constr. "+l(i,j)).c_str()));
				model.add(IloRange(env, 0, expr_r, 
						IloInfinity, ("right no.1 constr. "+l(i,j)).c_str()));
			}
		}
			
	    //in a right/left tour constr. no.2
	    int K = _inst.num_vehicles();
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				//build constraint
				IloExpr expr_l(env), expr_r(env);	
				expr_l += -K * vars[v[l(i,j)]];
				expr_r += -K * vars[v[r(i,j)]];
				for(unsigned int k=1; k<=_inst.num_vehicles(); ++k){
					expr_l -= IloInt(k)*vars[v[x(i,k)]] - IloInt(k)*vars[v[x(j,k)]];
					expr_r += IloInt(k)*vars[v[x(i,k)]] - IloInt(k)*vars[v[x(j,k)]];
				}	
			
				//add both contraints
				model.add(IloRange(env, 1-K, expr_l,
						IloInfinity, ("left no.2 constr. "+l(i,j)).c_str()));
				model.add(IloRange(env, 1-K, expr_r,
						IloInfinity, ("right no.2 constr. "+l(i,j)).c_str()));
			}
		}
		
		
		//same vehicle constraint z_i_j
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i.num()==j.num()) continue;
					model.add( vars[v[l(i,j)]] + vars[v[r(i,j)]]  +  vars[v[z(i,j)]]  == 1 );
			}
		}
		
		//distance between two successive jobs sufficient
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				IloExpr expr(env);
				model.add( -M*vars[v[z(i,j)]] - M*vars[v[y(i,j)]]  + vars[v[t(j)]]  -  vars[v[t(i)]] >= (-2*M + dist_inf<int>( i.get_beta(),j.get_alpha()) + i.length()) );
			}
		}
		
		//cheating:
		/*
		model.add( vars[v[t(1)]] == 10);
		model.add( vars[v[t(2)]] == 9);
		model.add( vars[v[x(1,1)]] == 1);
		model.add( vars[v[x(1,2)]] == 0);
		model.add( vars[v[x(2,1)]] == 0);
		model.add( vars[v[x(2,2)]] == 1);
		model.add( vars[v[y(1,2)]] == 1);
		model.add( vars[v[y(2,1)]] == 0);
		model.add( vars[v[x(1,1)]] == 1);
		model.add( vars[v[z(1,2)]] == 0);
		model.add( vars[v[r(1,2)]] == 0);
		model.add( vars[v[r(2,1)]] == 1);
		model.add( vars[v[l(1,2)]] == 1);
		model.add( vars[v[l(2,1)]] == 0);
		*/
}



void MIP::_build_variables_single_vehicle(IloEnv &env, IloNumVarArray &vars,IloModel &model){
	int counter = 0;
	
	//single makespan variable
	std::string name = "makespan";
	v[name] = counter++;
	vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
	
	for(const Job& i: _inst){
		//t: time variable
		std::string name = t(i);
		v[name] = counter++;
		vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
		
		
		//y: precedence variable(y_i_j)
		for(const Job& j: _inst){
			//skip variable y_i_i
			if(i.num()==j.num()) 
				continue;
			std::string name = y(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/, IloNumVar::Bool/*type*/, name.c_str() ) );
		}	
			
	}
	model.add(vars);
}


void MIP::_build_constraints_single_vehicle(IloEnv &env, IloNumVarArray &vars,IloModel &model){
		//makespan at least as late as last return of all vehivles
		for(const Job& j: _inst){
			IloExpr expr(env);
			expr += 1*vars[v["makespan"]];
			expr -= 1*vars[v[t(j)]];
			expr -= dist_inf<int>( j.get_beta(),_inst.get_depot(0) );    
			//IloRange(const IloEnv env, IloNum lhs, const IloNumExprArg expr, IloNum rhs, const char * name)
			model.add(IloRange(env, j.length(), expr, IloInfinity,("makespan setting for "+t(j)).c_str() ));
		}
		
			
		//precedence variable setting 1
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i.num()==j.num()) continue;
				model.add( M*vars[v[y(i,j)]] + vars[v[t(i)]]  -  vars[v[t(j)]]  >= 0 );
			}
		}
		
		//precedence variable setting 2
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i.num()==j.num()) continue;
				model.add( vars[v[y(i,j)]]  + vars[v[y(j,i)]] == 1 );
			}
		}
		
		//each job reachable from its depotposition in time
		for(const Job& j: _inst){
			IloExpr expr(env);
			expr += 1*vars[v[t(j)]];
			expr -=  dist_inf<int>( _inst.get_depot(0),j.get_alpha() ); 

			model.add(IloRange(env, 0, expr, IloInfinity, ("starttime reachable from depot for j_"+std::to_string(j.num())).c_str() ));
		}
		
		
		//distance between two successive jobs sufficient
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				IloExpr expr(env);
				model.add( - M*vars[v[y(i,j)]]  + vars[v[t(j)]]  -  vars[v[t(i)]] >= (-M + dist_inf<int>( i.get_beta(),j.get_alpha()) + i.length()) );
			}
		}
} 


void MIP::_parse_solution_single_vehicle(IloCplex &cplex, Tours &tours,IloNumVarArray &vars){
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[v[t(j)]], 0/*Index of solution in sol pool*/);
		tours.add_job(&j, tj, 0);
	}	
}





