#include "m_TSP_MIP.h"
#include "DisjointSet.h"
#include "MinCut.h"

#include <set>
#include <thread>
#include <algorithm>

using namespace std;

void m_TSP_MIP::add_objective_function_(){
	model_.add(IloMinimize(env_,vars_[v_["makespan"]]));
}
		 
//building variables
void m_TSP_MIP::build_variables_(){
	auto typeBool = IloNumVar::Bool;
	auto typeInt = IloNumVar::Int;
	if(LP_relaxation_){
		typeInt = IloNumVar::Float; 
		typeBool = IloNumVar::Float; 
	}


	//additional depot and 
	string name = "makespan";
	v_[name] = counter_++;
	vars_.add( IloNumVar(env_, 0/*lb*/, IloInfinity/*ub*/,typeInt, name.c_str() ) );

	//time variables
	for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
		name = name_t_(j);
		v_[name] = counter_++;
		vars_.add( IloNumVar(env_, 0/*lb*/, IloInfinity/*ub*/,IloNumVar::Float, name.c_str() ) );
	}
	
	//directed graph variables
    for(uint i = 1; i <= inst_.num_jobs()+inst_.num_vehicles();++i)
		for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
			//skip variable x_i_i, if not a depot
			 if(i==j and i<=inst_.num_jobs() )
				continue;
			string name = name_x_(i,j);
			v_[name] = counter_++;
			vars_.add( IloNumVar(env_, 0/*lb*/, 1/*ub*/, typeBool, name.c_str() ) );
		}

		
	//tour variable:
	for(uint j = 1; j <= inst_.num_jobs()+inst_.num_vehicles();++j){
		name = name_k_(j);
		v_[name] = counter_++;
		vars_.add( IloNumVar(env_, 1/*lb*/, inst_.num_vehicles()/*ub*/,typeInt, name.c_str() ) );
	}		
}

										
//constraint construction
void m_TSP_MIP::build_constraints_(){
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	
	if (bigM <= 0)
	    bigM = inst_.get_upper_bound();
    if(fixed_makespan_>0)
        bigM = fixed_makespan_;
	
	//add OUT degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint j = 1; j<=n+K; ++j){
			//TODO: is the Break here OK?
			if(i>n and j>n) break; //no edges from a depot to another depot
			if(i==j ) continue;
			expr += x(i,j);
		}

		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0,
									 ("out degree for "+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
		
	//add IN degree for vertices
	for(uint i = 1; i<=n+K; ++i){
		IloExpr expr(env_);
		for(uint j = 1; j<=n+K; ++j){
			//TODO: is the Break here OK?
			if(i>n and j>n) break; //no edges from a depot to another depot
 			if(i==j) continue; //add x_ii only for depot
			expr += x(j,i);
		}
		expr -= 1;	
		IloRange constraint(env_, 0, expr, 0, 
									("in degree for "+to_string(i)).c_str() );
		cons_.add(constraint);		
	}
	
		
	//starting time variables for depots
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( t(i) == 0);	
	
	//starting time variables for regular jobs
	for(uint i = 1; i<=n; ++i)
		for(uint j = 1; j<=n; ++j){
			if(i==j) continue;
			const Job& job_i = inst_[i-1];
			const Job& job_j = inst_[j-1];
		    
			auto tmpBigM = bigM+ job_i.length() + dist_inf(job_i.beta(),job_j.alpha());  
			cons_.add( t(j) - 1*t(i)- tmpBigM*x(i,j)  
							>=  job_i.length() + dist_inf(job_i.beta(),job_j.alpha())
								 -tmpBigM);
		}
	
	//starting time variables for "first" jobs after depots
	for(uint i = n+1; i<=n+K; ++i)
		for(uint j = 1; j<=n; ++j){
			if(i==j) continue;
			const Job& job_j = inst_[j-1];
			cons_.add( t(j)  - dist_inf(inst_.get_depot(i-n-1),job_j.alpha()) * x(i,j) >=0 );
		}	
	
	
	//makespan constraint for every job
	for(uint i=1; i<=n; ++i){
		IloNumVar &makespan = vars_[v_["makespan"]];
		const Job& job = inst_[i-1];
		IloExpr expr(env_);
		expr += 1*makespan;
		expr -= t(i);
		expr -= job.length();
		for(uint j = 1; j<=K; ++j)
			expr -= dist_inf(inst_.get_depot(j-1),job.get_beta()) * x(i,n+j);
			
		IloRange constraint(env_, 0, expr, IloInfinity, ("makespan cons due to vehicle "+to_string(i)).c_str() );
		cons_.add(constraint);	
	}

	//vehicle number for depots is there number
	for(uint i = n+1; i<=n+K; ++i)
		cons_.add( k(i) == i-n);
	
	//along edges the number has to stay the same!
	//from depot/job to job
	for(uint i = 1; i<=n+K; ++i)
		for(uint j = 1; j<=n+K; ++j){
			if(i>n and j > n) continue; //no x(i,j) between depots 
			if(i==j) continue;	
			IloExpr expr(env_);
			expr +=  k(i);
			expr -=  k(j);
			expr +=  static_cast<IloInt>(K)* x(i,j);
			
			IloRange constraint(env_, -IloInfinity, expr, K, ("vehicle assignment "+name_x_(i,j)).c_str() );
			cons_.add(constraint);	
		}

	//additional constraint to increase the makespan:
	//makespan >= each single tour
	if(tighten_){
		IloExpr expr(env_);
		//ave to sum upp all edges
		expr = static_cast<IloInt>(K) * vars_[v_["makespan"]];
		//inter job vertices
		for(uint i = 1; i<=n; ++i)
			for(uint j = 1; j<=n; ++j){
				if(i==j) continue;
		            const Job& j1 = inst_[i-1];
		            const Job& j2 = inst_[j-1];
		            int l = j1.length();
		            l += dist_inf(j1.beta(),j2.alpha());
		            
		            expr -=  l* x(i,j);
		    }
		//depot->job vertices, job->depot vertices        
		for(uint i = 1; i<=n; ++i){
			for(uint v = 1; v<=K; ++v){
	        	//to depot:
				const Job& j1 = inst_[i-1];
			    int l = j1.length();
		        //do not drive back to depot if not needed!
		        if(returning_to_depot_){
					l += dist_inf(j1.beta(),inst_.get_depot(v-1));	            
		            expr -=  l * x(i,n+v);
		        }
		        //from depot
			    l = dist_inf(inst_.get_depot(v-1),j1.alpha());
				expr -=  l * x(n+v,i);
			}
		}    
		IloRange constraint(env_, 0, expr, IloInfinity,
		"length_bound_for_tours");
		cons_.add(constraint);		
	}

}

	
//parsing of solution		
void m_TSP_MIP::parse_solution_(Tours &tours){
    //check wheter the model is capable of producing falid sol
	if( not collision_avoidance_ or LP_relaxation_)
		return;
	uint n = inst_.num_jobs();
	uint K = inst_.num_vehicles();
	//build all k tours
	for( uint i = 1; i<=K; ++i){
		uint current_pos = n+i;//starting at depot i
		do{
			//find next job
			for( uint j = 1; j<=n+K; ++j){
				if(current_pos==j) continue;
				//cout<< "trying "<<name_x_(current_pos,j)<< endl;
				if( cplex_.getValue( x(current_pos,j) ) > 0.5 ){
					//cout<< name_x_(current_pos,j) <<" = "<<cplex_.getValue( x(current_pos,j) ) << endl;
					double time = cplex_.getValue(t(j));
					if(j<=n){
						//cout<<"adding job "<<j<< endl;
						tours.add_job(&inst_[j-1], time,i-1);
					}
					current_pos = j;
					break;
				}	
			}
		}while(n+i!=current_pos);	
	}

	//cplex_.exportModel("mTSP.lp");
	//cplex_.writeSolutions("mTSP.sol"); 
}



//Cut callback 
//checks for fractional subtours, which means: ?????
ILOUSERCUTCALLBACK1(SubtourCutsMTSPCallback, m_TSP_MIP*, mip )
{       
   // Skip the separation if not at the end of the cut loop
   if( !isAfterCutLoop() )
      return;
    
 
	 
	uint n = mip->inst_.num_jobs();
 	uint K = mip->inst_.num_vehicles(); 

	//parse LP solution   
	IloEnv env = getEnv();
	IloInt numNodes =  mip->vars_.getSize();
	IloNumArray xSol(env, numNodes);
	getValues(xSol,  mip->vars_);

	auto arc = [&] (int i, int j){ return xSol[mip->v_[mip->name_x_(i,j)]];};
	//calculate min cut
	std::vector<std::tuple<int,int,double>> edges;
	//capacity of 2 between depots
	for(uint i=n+1; i<=n+K; ++i)
		for(uint j=i+1; j<=n+K; ++j){
			edges.push_back(make_tuple(i-1,j-1,2));
		}
	//ask once for every variable!	
	//add sum of arce in both dorections
	for(uint i=1; i<=n; ++i)
		for(uint j=i+1; j<=n+K;++j)
			if(arc(i,j)+arc(j,i)> 0.0001)	
				edges.push_back(make_tuple(i-1,j-1,arc(i,j)+arc(j,i)));
	std::vector<bool> cut = find_min_cut(edges,n+K); 


    assert(cut.size()==n+K or cut.size()==0);

    if(cut.size()==0){
    	xSol.end();
    	return;
    }
	IloExpr new_cut(env);

	for(uint i=1; i<=cut.size(); ++i)
		for(uint j=i+1; j<=cut.size(); ++j)
			if(cut[i-1] != cut[j-1]){
				//std::cout<< "add undir. arc "<<i<<"-"<<j <<std::endl;
				assert( i<=n or j<=n);//not two depots!
				new_cut += 1*mip->x(i,j);
				new_cut += 1*mip->x(j,i);
			}
	add(new_cut >= 2).end();
	new_cut.end();
   //free all cplex stuff
   xSol.end();
   return;
}


/**
 Own solve()-method to enable an own cut-callback
**/
std::pair<Tours,double> m_TSP_MIP::solve(){
	auto start = std::chrono::system_clock::now();
	Tours tours(inst_.num_vehicles());

	//build the MIP model and solve it!
	try {
	
		if(silent_){
			cplex_.setParam(IloCplex::MIPDisplay, 0);
			cplex_.setParam(IloCplex::SimDisplay, 0);
		}
		
		
		if(timelimit_ > 0)
			cplex_.setParam(IloCplex::TiLim, 60*timelimit_);

		build_variables_();
		
		if( 1 != inst_.num_vehicles() and  collision_avoidance_ )
			build_collision_variables_();
		
		model_.add(vars_);
			
		//objective function: minimize makespan
		if(fixed_makespan_ < 0)
		    add_objective_function_();
		
		build_constraints_();
	
		if( 1!= inst_.num_vehicles() and  collision_avoidance_)
			build_collision_constraints_();  

		model_.add(cons_);

       
		//run cplex and solve the model!
		cplex_.extract(model_);
		
		//write model to file
		//TODO: remove
		//cplex_.exportModel("tmp_mip.lp");
		
	    //print small models 
		if( debug_ and inst_.num_jobs()<=6)
			cout<<"MIP Model: \n"<<model_<<endl;
		
		//add callback to create cuts on the fly
		uint threads = thread::hardware_concurrency();
		//BUGFIX: enable it again
		if(use_subtour_cuts_)
			cplex_.use( SubtourCutsMTSPCallback(env_, this) );
		
		cplex_.setParam(IloCplex::IntParam::Threads	,threads);
		if(not silent_) cout<<"Detected "<<threads<<" cores" <<endl;
		
		if(not silent_ and debug_ and fixed_makespan_>0)
		    cout<< "Checking for solution with makespan at most "
		        << fixed_makespan_<< endl;
			
		bool solved = cplex_.solve();
		if(debug_) cout<< "Solving MIP was successful: "<<boolalpha<<solved<<endl;
		if(debug_) cout<< "MIP status = "<<cplex_.getStatus()<<endl;
		auto stop = std::chrono::system_clock::now();
		runningtime_ = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
		if ( !solved ) {
			env_.end();
			//return empty tours if MIP was unsolvable!
			return pair<Tours,double>{Tours(inst_.num_vehicles()),-1};
		}else{
			//parse solution
			//if(LP_relaxation_ and inst_.num_jobs()<10)
			//	print_LP_solution_();
			
			found_objective_ =  cplex_.getObjValue();
			parse_solution_(tours);
		}
		
		if(debug_)	print_LP_solution_();
		
	}catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "WARNING: Unknown exception caught while handling with CPLEX" << endl;
		
		exit(1); // Returns 1 to the operating system
	}
	//end it
	env_.end();


	return pair<Tours,double>(tours,found_objective_);
}	



//------ Here are some GTests for this class---//
#ifdef GTESTS_ENABLED
#include <gtest/gtest.h>
TEST(Two_Index_MIP, Normal) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    m_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(false);
	mip.set_tightening_cons(false);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ(objective, 221);
    EXPECT_TRUE (i.verify(t));
}

TEST(Two_Index_MIP, SubtourCuts) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    m_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(true);
	mip.set_tightening_cons(false);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ(objective, 221);
    EXPECT_TRUE(i.verify(t));

}

TEST(Two_Index_MIP, Tightening) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    m_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(false);
	mip.set_tightening_cons(true);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ (objective, 221);
    EXPECT_TRUE(i.verify(t));
}

TEST(Two_Index_MIP, TighteningAndCuts) { 
    Instance i; 
    i.set_num_vehicles(3);
    uint seed = 5; uint n = 8;
    i.generate_random_depots(0,100,0,20,seed);
    i.generate_random_jobs(n,0,100,0,20,seed);
    m_TSP_MIP mip(i);
    mip.set_silent(true);
    mip.set_collision(true); 
	//settings: 
	mip.use_subtour_cuts(true);
	mip.set_tightening_cons(true);
	auto mip_sol = mip.solve();
    Tours t = get<0>( mip_sol );
    double objective = get<1>( mip_sol );
    EXPECT_DOUBLE_EQ(objective, 221);
    EXPECT_TRUE (i.verify(t));
}

#else

#endif
