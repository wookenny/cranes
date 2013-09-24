#include "SingleCraneTSP_Solver.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>

using namespace std;

const string solver_call = "concorde"; 
const string tsp_file = "TMP_TSP_FILE.TSP"; 

Tours SingleCraneTSP_Solver::operator()(const Instance& inst) const{
	int M = inst.get_upper_bound();
	
	cout << "bound = " << M << endl;
	
	//create distance matrix
	int size = 3*(inst.num_jobs()+1);
	vector<vector<int>> dist;
	for(int i=0; i<size;++i)
	    dist.push_back( vector<int>(size,M) );    
	
	//iniatialize entries
    set_distances(dist,inst);
	    
	//write tsp file    
	create_TSP_file(dist);
	
	//solve it
	cout<< (solver_call+" "+tsp_file) << endl;
	int ret = system( (solver_call+" "+tsp_file).c_str() );
	if(ret!=0){
	    cerr<<"WARNING: Could not delete"<<tsp_file<<"!"<< endl;
	    return Tours{1};
	}    
	
	//delete everything
	//if( remove( tsp_file.c_str() ) != 0 )
	//    cerr<<"WARNING: Could not delete"<<tsp_file<<"!"<< endl;
    	
	
	Tours t{1};
	return t;
}

void SingleCraneTSP_Solver::create_TSP_file(const vector<vector<int>> &dist) const{
    fstream f;
    f.open( tsp_file.c_str(), ios::out );
    write_TSP_file(f, dist);
    f.close();
}

void SingleCraneTSP_Solver::write_TSP_file(fstream &file, const vector<vector<int>> &distances) const{
    file << "NAME : 2DVS Instance"<<endl;
    file << "TYPE : TSP" <<endl;
    file << "DIMENSION: "<< distances.size() <<endl;
    file << "EDGE_WEIGHT_TYPE : EXPLICIT" <<endl;
    file << "EDGE_WEIGHT_FORMT :  FULL_MATRIX" <<endl;
    file << "EDGE_WEIGHT_SECTION :" <<endl;
    for(const auto& row: distances){
        for(auto entry: row)
            file<<entry << " ";
        file<<endl;
    } 
    file << "EOF" <<endl;
}

void SingleCraneTSP_Solver::set_distances(vector<vector<int>> &dist, const Instance& inst) const{
    /* set distance acording to the scheme:
    every job is split up in the parts.
        - alpha, somewhere between and beta
        - verteices: depot, depot,depot, j1, j1,j1, j2,....
        => job i at position 3*i, 3i+1, 3i+2
        => index j belongs to job j/3, 
        - middle is only connected to alpha and beta
        - only alpha, beta connections
    distances are:
    */
    uint N = dist.size()/3;
	//set selfloop to 0
	for(uint i=0; i< dist.size(); ++i)
	    dist[i][i]=0;
	    
	//connect all middle jobs to left and right 
	dist[0][1] = dist[1][0] = dist[1][2] = dist[2][1] = 0;
	for(uint i=1; i<N;++i){
	    dist[3*i][3*i+1] = dist[3*i+1][3*i] = 0;
	    dist[3*i+1][3*i+2] = dist[3*i+2][3*i+1] = inst[i-1].length();
	    
	    for(uint j=i+1; j<N;++j){
	        auto& j1 = inst[i-1];
	        auto& j2 = inst[j-1]; 
	        dist[3*i][3*j+2] = dist[3*j+2][3*i] = dist_inf(j1.alpha(),j2.beta());
	        dist[3*i+2][3*j] = dist[3*j][3*i+2] = dist_inf(j1.beta(),j2.alpha());
	    }
	}

    //distance to/from depot
    for(uint i=1; i<N;++i){
        auto& j1 = inst[i-1];
        dist[0][3*i+2] = dist[3*i+2][0] = dist_inf(j1.beta(), inst.get_depot(0));
	    dist[2][3*i] = dist[3*i][2] = dist_inf(j1.alpha(), inst.get_depot(0));    
    }	
	        
}
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
