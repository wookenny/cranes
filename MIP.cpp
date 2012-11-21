#include "MIP.h"

#include <ilcplex/ilocplex.h>
#include <iostream>

#include "Job.h"
#include "Tours.h"

using namespace std;

ILOSTLBEGIN 

/***********************************************************************
		 functions to refer the different variables easily
***********************************************************************/
//These functors are used to create all functions automatically 
//to obey the DRY principle.


class f_two_parameter{
		string _name;
		public:
		f_two_parameter(string name):_name(name){}
		string operator()(int i, int j){
			assert(i!=j);
			return  string(_name)+"_"+to_string(i)+"_"+to_string(j);
		}
		string operator()(const Job& i, const Job& j){								
			return operator()( i.num(), j.num() );	
		} 
};

class f_single_parameter{
		string _name;
		public:
		f_single_parameter(string name):_name(name){}
		string operator()(int i){
			return  string(_name)+"_"+to_string(i);
		} 
		string operator()(const Job& i){												
			return operator()( i.num());													
		}
};


//x_i_k =1 => job i is processed by vehicle k
string x(int i, int k){
	return  "x_"+to_string(i)+"_"+to_string(k);
}
string x(const Job& i,int k){return x(i.num(),k);}

//z_i_j =1 => job i is handled by the same vehicle as job j
string z(int i, int j){
	assert(i!=j);
	if(i<j)
		return  "z_"+to_string(i)+"_"+to_string(j);
	else
		return  "z_"+to_string(j)+"_"+to_string(i);
}
string z(const Job& i,const Job& j){return z(i.num(),j.num());}

//starting time variable for job i
auto t = f_single_parameter("t");
//y_i_j =1 => job i starts earlier than job j
auto y = f_two_parameter("y");
//l_i_j =1 => job i is on vehicle left of job j
auto l = f_two_parameter("l");
//r_i_j =1 => job i is on vehicle right of job j
auto r = f_two_parameter("r");
//c_i_j =1 => job i has to be on a vehicle right of job j
auto c = f_two_parameter("c");
//other collisions functions
auto caa_p = f_two_parameter("caa_p");
auto caa_m = f_two_parameter("caa_m");
auto cab_p = f_two_parameter("cab_p");
auto cab_m = f_two_parameter("cab_m");
auto cba_p = f_two_parameter("cba_p");
auto cba_m = f_two_parameter("cba_m");
auto cbb_p = f_two_parameter("cbb_p");
auto cbb_m = f_two_parameter("cbb_m");


Tours MIP::solve(bool collision_free, bool LP_relax,bool debug){

	Tours tours(_inst.num_vehicles());

	//build the MIP model and solve it!
	IloEnv env;
	try {
		IloModel model(env);
		IloCplex cplex(env);
		IloNumVarArray vars(env);		
		
		if(_inst.num_vehicles()==1)
			_build_variables_single_vehicle(env, vars, LP_relax);
		else
			_build_variables(env, vars, LP_relax);
		
		if( 1!=_inst.num_vehicles() and  collision_free)
			_build_collision_variables(env, vars, LP_relax);
		
		model.add(vars);
			
		//objective function: minimize makespan
		model.add(IloMinimize(env,vars[v["makespan"]]));	
		
		if(_inst.num_vehicles()==1)
			_build_constraints_single_vehicle(env,vars,model);
		else		
			_build_constraints(env,vars,model);
	
		if( 1!=_inst.num_vehicles() and  collision_free)
			_build_collision_constraints(env, vars, model);  
			
		//Tests with some constraints to tighten teh LP relaxation	
		if( 1!=_inst.num_vehicles() and  LP_relax )
			_build_tightening_contraints(env, vars, model);
			
		
		//run cplex and solve the model!
		cplex.extract(model);
		
		//print small models 
		if(_inst.num_vehicles()<=6)
			cout<<"MIP Model: \n"<<model<<endl;
		
		bool solved = cplex.solve();
		if ( !solved ) {
			env.end();
			return Tours(_inst.num_vehicles());
		}else{
			//parse solution
			//return empty tour id only an LP relaxation was solved
			if(LP_relax){
				_print_LP_solution(cplex,vars);
				return tours;
			}	
			if(_inst.num_vehicles()==1)
				_parse_solution_single_vehicle(cplex, tours,vars,debug);
			else{
				_parse_solution(cplex, tours,vars,debug);
				}
		}
		
		

		
	}catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "WARNING: Unknown exception caught while handling with CPLEX" << endl;
		exit(1); // Returns 1 to the operating system
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
	if(not debug and _inst.num_vehicles() > 5) return;
	
	//print additional variables for debug informations
	cout<< "-------------DEBUG HINTS-------------"<<endl;
	cout<< "starting times and assignment variables:"<<endl;
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[v[t(j)]]);
		cout<<"t_"<<j.num()<<" = "<<tj<<"\t";	
		for(unsigned int k=1;k<=_inst.num_vehicles(); ++k){
			if(cplex.getValue(vars[v[x(j,k)]])>=0.1){
				cout<< "veh. "<<k<<endl;
			}
		}	
	}
	
	cout<< "\nprecedence variables:"<<endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i==j) continue;
			if(cplex.getValue(vars[v[y(i,j)]])>0.1)
				cout<< y(i,j) << " = "<<	cplex.getValue(vars[v[y(i,j)]])<<endl;
		}
	}
	
	cout<< "\nsame vehicle variables:"<<endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i.num()>=j.num()) continue;
			if(cplex.getValue(vars[v[z(i,j)]])>0.1)
				cout<< z(i,j) << " = "<<	cplex.getValue(vars[v[z(i,j)]])<<endl;
		}
	}
	
	cout<< "\nL/R variables:"<<endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i==j) continue;
			if(cplex.getValue(vars[v[l(i,j)]])>0.1)
				cout<< l(i,j) << " = "<<	cplex.getValue(vars[v[l(i,j)]])<<endl;
			if(cplex.getValue(vars[v[r(i,j)]])>0.1)
				cout<< r(i,j) << " = "<<	cplex.getValue(vars[v[r(i,j)]])<<endl;	
		}
	}
	cout<< "-------------END OF DEBUG HINTS-------------"<<endl;
	
}

/**Adding all necessary variables to the var array and an additional redirect in the map to use the nicer name of the
variables for direct access.
**/
void MIP::_build_variables(IloEnv &env, IloNumVarArray &vars, bool LP_relax){
	auto type = IloNumVar::Bool;
	if(LP_relax)
		type = IloNumVar::Float; 
		
	//single makespan variable
	string name = "makespan";
	v[name] = counter++;
	vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
	
	for(const Job& i: _inst){
		//t: time variable
		string name = t(i);
		v[name] = counter++;
		vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
		
		//x: assignment variable(x_i_j=1 => job i done by vehicle j)
		for(unsigned int k=1; k<=_inst.num_vehicles(); ++k){
			string name = x(i,k);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/,1/*ub*/, type, name.c_str() ) );
		}

		
		//y: precedence variable(y_i_j)
		for(const Job& j: _inst){
			//skip variable y_i_i
			if(i.num()==j.num()) 
				continue;
			string name = y(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
		}	
		
		//z: same vehicle variable(z_i_j)		
		for(const Job& j: _inst){					
			//only variables z_i_j for i < j
			if(i.num()>=j.num()) 
				continue;
			string name = z(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
		}
		
		//r: vehicle more to the right(r_i_j)
		for(const Job& j: _inst){
			//skip variable r_i_i
			if(i.num()==j.num()) 
				continue;
			string name = r(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
		}
		
		//l: vehicle more to the left(l_i_j)
		for(const Job& j: _inst){
			//skip variable l_i_i
			if(i.num()==j.num()) 
				continue;
			string name = l(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/,1/*ub*/, type, name.c_str() ) );
		}
				
	}
} 


void MIP::_build_collision_variables(IloEnv &env, IloNumVarArray &vars, bool LP_relax){
	auto type = IloNumVar::Bool;
	if(LP_relax)
		type = IloNumVar::Float; 

	for(const Job&  i: _inst){
		for(const Job&  j: _inst){
			//skip variable r_i_i
			if(i.num()==j.num()) 
				continue;
			
			//c: chain variable(c_i_j)	
			string name = c(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
	
			//c_xx_y: helper variables for the chain var	
			auto names = { caa_p(i,j), caa_m(i,j), cab_p(i,j),cab_m(i,j), cba_p(i,j), cba_m(i,j), cbb_p(i,j), cbb_m(i,j) };
			for( string var_name : names ) {
				v[var_name] = counter++;
				vars.add( IloNumVar(env, 0/*lb*/, 1/*ub*/, type, var_name.c_str() ) );	
			}
		}
	}
							
}


void MIP::_build_constraints(IloEnv &env, IloNumVarArray &vars,IloModel &model){

		//makespan at least as late as last return of all vehicles
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
			model.add(IloRange(env, 1, expr, 1, ("assignemt to vehicle for j_"+to_string(j.num())).c_str() ));
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
			model.add(IloRange(env, 0, expr, IloInfinity, ("starttime reachable from depot for j_"+to_string(j.num())).c_str() ));
		}
		
		//in a right/left tour constr. no.1
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				//build constraint
				IloExpr expr_l(env), expr_r(env);	
				expr_l += IloInt(_inst.num_vehicles()-1) * vars[v[l(i,j)]];
				expr_r += IloInt(_inst.num_vehicles()-1) * vars[v[r(i,j)]];
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
				model.add( -M*vars[v[z(i,j)]] - M*vars[v[y(i,j)]]  + vars[v[t(j)]]  -  vars[v[t(i)]] >= (-2*M + dist_inf<int>( i.get_beta(),j.get_alpha()) + i.length()) );
			}
		}
		
}

void MIP::_build_collision_constraints(IloEnv&, IloNumVarArray &vars, IloModel &model){										
		
		// c_i_j = 1 if one on the +/- pairs sums uo to two
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i.num()==j.num()) continue;
				//
				model.add( vars[v[c(i,j)]]  >= vars[v[caa_p(i,j)]]  +  vars[v[caa_m(i,j)]] - 1);
				model.add( vars[v[c(i,j)]]  >= vars[v[cab_p(i,j)]]  +  vars[v[cab_m(i,j)]] - 1);
				model.add( vars[v[c(i,j)]]  >= vars[v[cba_p(i,j)]]  +  vars[v[cba_m(i,j)]] - 1);
				model.add( vars[v[c(i,j)]]  >= vars[v[cbb_p(i,j)]]  +  vars[v[cbb_m(i,j)]] - 1);
			}
		}
			
		//c_xx_y: correct setting for all helping variables	
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				//build constraints
				model.add( M*vars[v[caa_p(i,j)]]  >= vars[v[t(j)]] - j.get_alpha()[0] - vars[v[t(i)]] + i.get_alpha()[0]);
				model.add( M*vars[v[caa_m(i,j)]]  >= vars[v[t(j)]] + j.get_alpha()[0] - vars[v[t(i)]] - i.get_alpha()[0]);		
				model.add( M*vars[v[cab_p(i,j)]]  >= vars[v[t(j)]] + j.length() - j.get_beta()[0] - vars[v[t(i)]] + i.get_alpha()[0]);
				model.add( M*vars[v[cab_m(i,j)]]  >= vars[v[t(j)]] + j.length() + j.get_beta()[0] - vars[v[t(i)]] - i.get_alpha()[0]);
				model.add( M*vars[v[cba_p(i,j)]]  >= vars[v[t(j)]] - j.get_alpha()[0] - vars[v[t(i)]] - i.length() + i.get_beta()[0]);
				model.add( M*vars[v[cba_m(i,j)]]  >= vars[v[t(j)]] + j.get_alpha()[0] - vars[v[t(i)]] - i.length() - i.get_beta()[0]);
				model.add( M*vars[v[cbb_p(i,j)]]  >= vars[v[t(j)]] + j.length() - j.get_beta()[0] - vars[v[t(i)]] - i.length() + i.get_beta()[0]);
				model.add( M*vars[v[cbb_m(i,j)]]  >= vars[v[t(j)]] + j.length() + j.get_beta()[0] - vars[v[t(i)]] - i.length() - i.get_beta()[0]);
			
				//
				model.add( vars[v[c(j,i)]]  + vars[v[c(i,j)]] <= 1);
				model.add( vars[v[l(i,j)]]  >= vars[v[c(i,j)]] );
			}
		}
		
		
		// c_i_j = 1 => assign j right of i
		/*int K = _inst.num_vehicles();
		for(const Job& i: _inst){
			for(const Job& j: _inst){
				if(i==j) continue;
				//build constraint
				IloExpr expr(env);
				expr += -(K);
				expr += +(K-1) * vars[v[c(i,j)]];
				
				for(int k=1; k<=K;++k){
					expr += k * vars[v[x(j,k)]];
					expr -= k * vars[v[x(i,k)]];
				}				
				//add contraint
				model.add(IloRange(env, 0, expr, IloInfinity, ("increasing chain "+c(i,j)).c_str()));
			}
		}
		*/
}
											

void MIP::_build_tightening_contraints(IloEnv&, IloNumVarArray &vars,
										IloModel &model){
	/*
	The idea:
	x_ik +l_ik-z_ij <= 1, for all vehicles k
	This means, if i and j are on the same vehicle,
	z_ij has to be one. For non-integral values for l_ij and r_ij  
	this is not enforced so far. 										
	*/	
	for(const Job& i: _inst)
			for(const Job& j: _inst){
				if(i==j) continue;
				for(uint k=1; k<=_inst.num_vehicles();++k){	
						model.add( vars[v[x(i,k)]] + vars[v[x(j,k)]]
									- vars[v[z(i,j)]]<= 1);
					}
			}
										
}
		
											
											

void MIP::_build_variables_single_vehicle(IloEnv &env, IloNumVarArray &vars, bool LP_relax){

	auto type = IloNumVar::Bool;
	if(LP_relax)
		type = IloNumVar::Float; 
		
	int counter = 0;
	
	//single makespan variable
	string name = "makespan";
	v[name] = counter++;
	vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
	
	for(const Job& i: _inst){
		//t: time variable
		string name = t(i);
		v[name] = counter++;
		vars.add( IloNumVar(env, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
		
		
		//y: precedence variable(y_i_j)
		for(const Job& j: _inst){
			//skip variable y_i_i
			if(i.num()==j.num()) 
				continue;
			string name = y(i,j);
			v[name] = counter++;
			vars.add( IloNumVar(env, 0/*lb*/, 1/*ub*/, type, name.c_str() ) );
		}	
			
	}
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

			model.add(IloRange(env, 0, expr, IloInfinity, ("starttime reachable from depot for j_"+to_string(j.num())).c_str() ));
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


void MIP::_parse_solution_single_vehicle(IloCplex &cplex, Tours &tours,
												IloNumVarArray &vars,bool debug){
	
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[v[t(j)]]);
		tours.add_job(&j, tj, 0);
	}	
	
	//if nothing to debug, we can finish here
	if(not debug) return;
	
	//print additional variables for debug informations
	cout<< "-------------DEBUG HINTS-------------"<<endl;
	cout<< "starting times and precedence variables:"<<endl;
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[v[t(j)]]);
		cout<<"t_"<<j.num()<<" = "<<tj<<"\t";	
	}
	
	cout<< "\nprecedence variables:"<<endl;
	for(const Job& i: _inst){
		for(const Job& j: _inst){
			if(i==j) continue;
			if(cplex.getValue(vars[v[y(i,j)]])>0.01)
				cout<< y(i,j) << " = "<<	cplex.getValue(vars[v[y(i,j)]])<<endl;
		}
	}
	cout<< "-------------END OF DEBUG HINTS-------------"<<endl;
}


void MIP::_print_LP_solution(const IloCplex &cplex,const IloNumVarArray &vars) const{
	cout<< "---- Variable values in the LP solution: ----"<<endl;
	cout<<"starting times:"<<endl;
	cout.precision(3);
	for(const Job& j: _inst){
		double tj = cplex.getValue(vars[ v.at(t(j))]);
		cout<<t(j)<<" = "<<tj<<endl;
	}	
	cout<<endl;
	
	for(const Job& j: _inst){
		for(unsigned int k=1; k<= _inst.num_vehicles(); ++k)
			if (cplex.getValue(vars[ v.at(x(j,k))]) > 0.01)
				cout<<x(j,k)<<" = " <<cplex.getValue(vars[ v.at(x(j,k))]) <<"\t";
		
		cout<<endl;
	}	
	cout<<endl;
	
	for(const Job& i: _inst)
		for(const Job& j: _inst){
			if(i==j) continue;
			cout<< l(i,j)<<" = "<<cplex.getValue(vars[ v.at(l(i,j))]);
			cout<<"\t"<<r(i,j) << " = "<<cplex.getValue(vars[ v.at(r(i,j))]);
			cout<<"\t"<<z(i,j) << " = "<<cplex.getValue(vars[ v.at(z(i,j))]);
			if(false)//TODO: I should know wether the c variables are present or not 
					//add many more class variables
				cout<<"\t"<<c(i,j) << " = "<<cplex.getValue(vars[ v.at(c(i,j))]);
			
			cout<<endl;
		}
	cout<<endl;
	
	
	cout<< "---- End of LP solution: ----"<<endl;
}





