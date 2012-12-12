#pragma once

#include "generalizedVRP_MIP.h"


class m_TSP_MIP: public generalizedVRP_MIP{
	
		
	private:
		
		stringify name_t_; 
		variable_access t;
		stringify name_x_; 
		variable_access x;
		stringify name_k_; 
		variable_access k;
		int bigM;
	
	public:
	
		virtual ~m_TSP_MIP(){}
		m_TSP_MIP()=delete;
		m_TSP_MIP(const Instance& i): generalizedVRP_MIP(i), 
									  name_t_("t",1),
									  t(name_t_,vars_,v_),
									  name_x_("x",2),
									  x(name_x_,vars_,v_),
									  name_k_("k",1),
									  k(name_k_,vars_,v_){ }
		
	
		
	protected:	
		void add_objective_function_();
		
		//building variables
		void build_variables_(); 
		void build_collision_variables_();	
										
		//constraint construction
		void build_constraints_();
		void build_collision_constraints_();  
		
		//parsing of solution		
		void parse_solution_(Tours &tours);		


};
