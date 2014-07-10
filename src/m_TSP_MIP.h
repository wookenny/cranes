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
\sum_{i=1}^{n+k} x_i,j = 1			for all j \in [n+k]//indegree
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

class m_TSP_MIP: public generalizedVRP_MIP{
	
		
	private:
		
		stringify name_x_; 
		variable_access x;
		
	
	public:
	
		virtual ~m_TSP_MIP(){}
		m_TSP_MIP()=delete;
		m_TSP_MIP(const Instance& i): generalizedVRP_MIP(i), 
									  name_x_("x",2),
									  x(name_x_,vars_,v_){}
		std::pair<Tours,double> solve();				  

		friend class SubtourCutsMTSPCallbackI;
		
	protected:	
		void add_objective_function_();
		
		//building variables
		void build_variables_(); 
										
		//constraint construction
		void build_constraints_();
		
		//parsing of solution		
		void parse_solution_(Tours &tours);		


};
