#pragma once

#include <unordered_map>
#include <string>
#include <ilcplex/ilocplex.h>
#include <iostream>

#include "Instance.h"
#include "Job.h"
class Tours;


class generalizedVRP_MIP{

	
	protected:
		/* some helper classes for easy accessing of variables */
		class stringify{
			std::string s_;
			const int num_parameter_;
			public:
			stringify(const std::string s, int param):s_(s),num_parameter_(param){}
			
			std::string operator()(int i,int j,int k) const {
					assert(3==num_parameter_);
					return s_+"_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(k);}
			std::string operator()(const Job& i, const Job& j, const Job& k) const{
					return operator()(i.num(), j.num(), k.num());
			} 
			
			std::string operator()(int i,int j) const {
					assert(2==num_parameter_);
					return s_+"_"+std::to_string(i)+"_"+std::to_string(j);}
			std::string operator()(const Job& i, const Job& j) const{
					return operator()(i.num(), j.num());
			} 	
			
			std::string operator()(int i) const {
					assert(1==num_parameter_);
					return s_+"_"+std::to_string(i);}
			std::string operator()(const Job& i) const{
					return operator()(i.num());
			} 		
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
				IloNumVar& operator()(int i) const{
						assert(pos_.find(name_(i))!=pos_.end());
						return vars_[pos_[name_(i)]]; //at() because [] inserts element
				}
				IloNumVar& operator()(const Job& i) const{
						assert(pos_.find(name_(i))!=pos_.end());
						return vars_[pos_[name_(i)]]; 
				}
				IloNumVar& operator()(int i,int j) const{
						assert(pos_.find(name_(i,j))!=pos_.end());
						return vars_[pos_[name_(i,j)]]; 
				}
				IloNumVar& operator()(const Job& i,const Job& j) const{
						assert(pos_.find(name_(i,j))!=pos_.end());
						return vars_[pos_[name_(i,j)]]; 
				}
				IloNumVar& operator()(int i,int j,int k) const{
						assert(pos_.find(name_(i,j,k))!=pos_.end());
						return vars_[pos_[name_(i,j,k)]]; 
				}
				IloNumVar& operator()(const Job& i,const Job& j,const Job& k) const{
						assert(pos_.find(name_(i,j,k))!=pos_.end());
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
		IloRangeArray cons_;
		IloModel model_;
		IloCplex cplex_;
		
		//MIP specific flags
		bool debug_;
		bool collision_avoidance_;
		bool LP_relaxation_;
		
	public:
		generalizedVRP_MIP()=delete;
		generalizedVRP_MIP(const Instance& i):inst_(i),counter_(0),env_(),
											  vars_(env_), cons_(env_),
											  model_(env_), 
											  cplex_(env_), debug_(false),
											  collision_avoidance_(false),
											  LP_relaxation_(false){};
		virtual ~generalizedVRP_MIP(){};
		Tours solve();


		void set_debug(bool v){debug_=v;}
		void set_LP(bool v){LP_relaxation_ = v;}
		void set_collision(bool v){collision_avoidance_ = v;}		

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

