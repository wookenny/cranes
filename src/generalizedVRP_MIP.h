#pragma once

#include <unordered_map>
#include <string>
#include <utility> 
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
				     //if( pos_.find(name_(i,j))==pos_.end() )
				     //    std::cout << name_(i,j) << std::endl;
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
		bool silent_ = false;
		bool use_subtour_cuts_ = false;

		int bigM =0;
		int fixed_makespan_;
		
		double found_objective_ = -1;
		//variables and helper for collision handling
		//collision name/variable helper
		stringify name_t_; 
		variable_access t;
		stringify name_k_; 
		variable_access k;
		stringify name_c_;
		variable_access c;	
		stringify name_caa_p_;
		variable_access caa_p;
		stringify name_caa_m_;
		variable_access caa_m;
		stringify name_cab_p_;
		variable_access cab_p;		
		stringify name_cab_m_;
		variable_access cab_m;		
		stringify name_cba_p_;
		variable_access cba_p;	
		stringify name_cba_m_;
		variable_access cba_m;		
		stringify name_cbb_p_;
		variable_access cbb_p;		
		stringify name_cbb_m_;
		variable_access cbb_m;

		
	public:
		generalizedVRP_MIP()=delete;
		generalizedVRP_MIP(const Instance& i):inst_(i),counter_(0),env_(),
											  vars_(env_), cons_(env_),
											  model_(env_), 
											  cplex_(env_), debug_(false),
											  collision_avoidance_(false),
											  LP_relaxation_(false),
											  fixed_makespan_(-1),
											  name_t_("t",1),
									  		  t(name_t_,vars_,v_),
											  name_k_("k",1),
									  		  k(name_k_,vars_,v_),
											  name_c_("c",2),
									  		  c(name_c_,vars_,v_),
									  		  name_caa_p_("c_aa_p",2),
									  		  caa_p(name_caa_p_,vars_,v_),
											  name_caa_m_("c_aa_m",2),
											  caa_m(name_caa_m_,vars_,v_),
											  name_cab_p_("c_ab_p",2),
											  cab_p(name_cab_p_,vars_,v_),
											  name_cab_m_("c_ab_m",2),
											  cab_m(name_cab_m_,vars_,v_),
											  name_cba_p_("c_ba_p",2),
											  cba_p(name_cba_p_,vars_,v_),
											  name_cba_m_("c_ba_m",2),
											  cba_m(name_cba_m_,vars_,v_),
											  name_cbb_p_("c_bb_p",2),
											  cbb_p(name_cbb_p_,vars_,v_),
											  name_cbb_m_("c_bb_m",2),
											  cbb_m(name_cbb_m_,vars_,v_){};
											  
		virtual ~generalizedVRP_MIP(){};
		virtual std::pair<Tours,double> solve();

		void set_debug(bool v){debug_=v;}
		void set_LP(bool v){LP_relaxation_ = v;}
		void set_collision(bool v){collision_avoidance_ = v;}
		void set_silent(bool s){silent_ = s;}			
        void set_fixed_makespan(int makespan){fixed_makespan_ = makespan; bigM = makespan;}
		virtual void use_subtour_cuts(bool c){use_subtour_cuts_ = c;}
		friend class SubtourCutsCallbackI;
    
        virtual void set_start_solution(const Tours &tours);       
        
	protected:
		//almost all versions are purely virtual because often the methods are 
		//depending heavily on the exact model. 
		
		virtual void add_objective_function_() = 0;
		
		//building variables
		virtual void build_variables_() = 0; 
		virtual void build_collision_variables_();	
										
		//constraint construction
		virtual void build_constraints_() = 0;
		virtual void build_collision_constraints_();  
		
		//parsing of solution		
		virtual void parse_solution_(Tours &tours) = 0;
		//generic version....further refinement possible!							
		virtual void print_LP_solution_() const; 		
		
};

