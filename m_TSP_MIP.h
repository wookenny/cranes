#pragma once

#include "generalizedVRP_MIP.h"

/***
This MIP solves the 2DVS with a multiple TSP formulation.
The MIP looks like this:

variables:
makespan,	a variable for the makespan
t_j, 		starting times for all jobs and all depots
x_i,j,		(binary)variables for directed edges between all jobs and all depot positions
tour_j,		(integral[0,k-1])tour variable, which assigns a job to a vehicle


constraints:
\sum_{j=1}^{n+k} x_i,j = 1 			for all i \in [n+k]//outdegree
\sum_{i=1}^{n+k} x_i,j = 1			for all j \in [n+k]//indrgree
t_{n+i} = 0							for all i \in [k]//start at depot at time 0

t(j) - bigM*x(i,j)  >= 							//regular starting time constraint
t(i)+ length(i) + dist(i,j) * x(i,j) - bigM)	//if(x_ij==1), startingtime is bigger
												//than first reachable time

t(j) >= dist(depot(i),j) * x(n+i,j)	// if edge from depot i to job j, then select correct starting time


 makespan >= t(i) + length(i)+dist_inf(depot(j),i) * x(i,n+j)
 //makespan is bigger than latest job and return to its depot
 
 tour_(n+i) =i //assign  depot i to tour i
 
  tour(i) -  tour(j) <=  K*(1- x(i,j)) // if x(i,j)=1 => vehicle difference <=0 in both direction! 
**/

//TODO: HINT: save the tour variable and add multiple edges, each for a single vehicle
//TODO: BigM besser initialisieren
class m_TSP_MIP: public generalizedVRP_MIP{
	
		
	private:
		
		stringify name_t_; 
		variable_access t;
		stringify name_x_; 
		variable_access x;
		stringify name_k_; 
		variable_access k;
		
		//collision name/variable helper
		//TODO: nach gen.VRPMIP verschieben udn per Macro initialisierem!
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
									  cbb_m(name_cbb_m_,vars_,v_){}				  

		
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
