#pragma once

#include "generalizedVRP_MIP.h"


class Assigment_MIP: public generalizedVRP_MIP{
	
		
	private:
		stringify name_x_;  //x_i_k = 1 => i is assigned to vehicle k
		variable_access x;
		stringify name_s_;  //s_i_j = 1 => i is in the same vehicle as j
		variable_access s; 
		stringify name_t_;  //t_i : 		starting time of job i
		variable_access t;
		stringify name_y_;  //y_i_j = 1 => job i is started before job j
		variable_access y;
		stringify name_r_;  //r_i_j = 1 => job i is right of job j
		variable_access r;  //(in sense of vehicle assignment) 
		stringify name_l_;  //l_i_j = 1 => job i is left ob job j 
		variable_access l; //(in sense of vehicle assignment) 
		int bigM;
	
	public:
	
		virtual ~ Assigment_MIP(){}
		Assigment_MIP()=delete;
		Assigment_MIP(const Instance& i): generalizedVRP_MIP(i), 
									  name_x_("x",2),
									  t(name_t_,vars_,v_),
									  ){ }
		
	
		
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
