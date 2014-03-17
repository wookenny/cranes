#pragma once

#include "generalizedVRP_MIP.h"

/***
This MIP solves the 2DVS with a multiple TSP formulation.
It's similar to the m_TSP_MIP but it handles all tours with 
a 3-indexed formulation.


The MIP looks like this:

variables:
makespan,	a variable for the makespan
t_j, 		starting times for all jobs and all depots
x_i,j,k		(binary)variables for directed edges between all jobs and all depot positions
			used by vehicle k
k_j,		(integral[1,k])tour variable, which assigns a job to a vehicle


constraints:
\sum_{j=1}^{n+k} x_i,j,v = \sum_{j=1}^{n+k} x_j,i,v  
		for all i \in [n+k] and for all v \in [k]//in=outdegree for non-depot nodes
				
				
				
\sum_{v=1}^{k}\sum_{j=1}^{n+k} x_i,j,k = 1 		for all i \in [n]//outdegree(w.o. depot)
\sum_{v=1}^{k}\sum_{i=1}^{n+k} x_i,j,k = 1		for all j \in [n]//indegree(w.o. depot)	
		
		
t_{n+i} = 0							for all i \in [k]//start at depot at time 0


\

t(j) - bigM*x(i,j,k)  >= 							//regular starting time constraint
t(i)+ length(i) + dist(i,j) - bigM)	  for all v \in [k]

alternativ: x(i,j,k) * (t(j) - t(i)- length(i) - dist(i,j)) >= 0  

//if(x_ij==1), startingtime is bigger
//than first reachable time

t(j) >= dist(depot(v),j) * x(n+v,j,v)	// if edge from depot v to job j, then select correct starting time


 makespan >= t(i) + length(i)+  \sum_{v=1}^{k} dist_inf(depot(v),i) * x(i,n+v,v)
 //makespan is bigger than latest job and return to its depot
 
 tour_(n+i) = i //assign  depot i to tour i
 tour(i) = \sum_{v=1}^{k} v * \sum_{j=1}^{n+k} x_j,i,v   
***/
class independent_TSP_MIP: public generalizedVRP_MIP{
	
	private:
		
		stringify name_x_; 
		variable_access x;
		
		Tours start;

        //set up astarting solution
	    void add_MIP_start_();
	
	public:
	
		virtual ~independent_TSP_MIP(){}
		independent_TSP_MIP()=delete;
		independent_TSP_MIP(const Instance& i): generalizedVRP_MIP(i), 
									  name_x_("x", 3),
									  x(name_x_,vars_,v_),
									  start(i.num_vehicles()){}				  
		std::pair<Tours,double> solve();

        virtual void set_start_solution(const Tours &tours){ start = tours; 
                                                        inst_.makespan(start);}

                                                      
		
		friend class SubtourCutsCallbackI;
		
	protected:	
		void add_objective_function_();
		
		//building variables
		void build_variables_(); 
										
		//constraint construction
		void build_constraints_();
		
		//parsing of solution		
		void parse_solution_(Tours &tours);		
        
};

