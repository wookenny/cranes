#pragma once

#include <unordered_map>
#include <string>
#include <ilcplex/ilocplex.h>
#include <iostream>

#include "Instance.h"

class Tours;
class Job;

class generalizedVRP_MIP{

	
	protected:
		/* some helper classes for easy accessing of variables*/
		class stringify{
			std::string s_;
			const int num_parameter_;
			public:
			stringify(const std::string s, int param):s_(s),num_parameter_(param){}
			
			std::string operator()(int i,int j,int k) const {
					assert(3==num_parameter_);
					return s_+"_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(k);}
			
			std::string operator()(int i,int j) const {
					assert(2==num_parameter_);
					return s_+"_"+std::to_string(i)+"_"+std::to_string(j);}
			
			std::string operator()(int i) const {
					assert(1==num_parameter_);
					return s_+"_"+std::to_string(i);}
					
		};
	
		class variable_access{
			private:
				stringify& name_;
				IloNumVarArray& vars_;
				std::unordered_map<std::string,int>& pos_;
			public: 
				variable_access(stringify& name,
								IloNumVarArray& vars,
								std::unordered_map<std::string,int>& pos):
								name_(name),vars_(vars),pos_(pos){}
				IloNumVar& operator()(int i){
						return vars_[pos_[name_(i)]]; 
				}
				IloNumVar& operator()(int i,int j){
						return vars_[pos_[name_(i,j)]]; 
				}
				IloNumVar& operator()(int i,int j,int k){
						return vars_[pos_[name_(i,j,k)]]; 
				}
			};
	
		//problem instance
		const Instance& inst_;
		
		//MIP data types
		std::unordered_map<std::string,int> v_;
		int counter_;
		IloEnv env_;
		IloNumVarArray vars_;
		IloModel model_;
		IloCplex cplex_;
		
		//MIP specific flags
		bool debug_;
		bool collision_avoidance_;
		bool LP_relaxation_;
		
	public:
		generalizedVRP_MIP()=delete;
		generalizedVRP_MIP(const Instance& i):inst_(i),env_(){};
		virtual ~generalizedVRP_MIP(){};
		Tours solve();

		

	protected:
		//almost all versions are purely virtual because often the methods are 
		//depending heavily on the exact model. 
		
		virtual void add_objective_function_() = 0;
		
		//building variables
		virtual void build_variables_() = 0; 
		virtual void build_collision_variables_() = 0;	
										
		//constraint construction
		virtual void build_constraints_() = 0;
		virtual void build_collision_constraints_() = 0;  
		
		//parsing of solution		
		virtual void parse_solution_(Tours &tours) = 0;		
		virtual void print_LP_solution_() const; //generic version....further refinement possible!							
};

