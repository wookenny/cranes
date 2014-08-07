#include "SingleCraneTSP_Solver.h"
#include "InsertionHeuristic.h"

#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "Common.h"

#include <boost/filesystem.hpp>
using namespace std;


//TODO: Count edges for bound!
// use insertion heuristic for better tour!

const string solver_call = "concorde"; 


SingleCraneTSP_Solver::SingleCraneTSP_Solver(){
    //initialize helpfull lambda function to handle positions correctly
    depot_start   =  [](int i){return 2*i;};
    depot_middle  =  [this](int i){return 2*K+3*N +i;};
    depot_end     =  [](int i){return 2*i+1;};
    job_start     =  [this](int i){return 2*K+3*i;};
    job_middle    =  [this](int i){return 2*K+3*i+1;};
    job_end       =  [this](int i){return 2*K+3*i+2;};
}

tuple<double, Tours> SingleCraneTSP_Solver::operator()(const Instance& inst, 
                                                        bool no_solution) const{
    //find new filenames    
    std::string random_name = boost::filesystem::unique_path("TSP_temp_file_%%%%-%%%%-%%%%-%%%%").native();
    tsp_file     = random_name+".tsp";
    tsp_sol_file = random_name+".sol";

    N = inst.num_jobs();
    K = inst.num_vehicles();
	int M = inst.get_upper_bound();
	//cout << "bound = " << M << endl;
	
	//create distance matrix
	//3 vertices for each jobs, 2 for every depot + 1 between neighboring depots
	int size = 3*inst.num_jobs() + 2*inst.num_vehicles() + inst.num_vehicles();
	vector<vector<int>> dist;
	for(int i=0; i<size;++i)
	    dist.push_back( vector<int>(size,M) );    
	
	//iniatialize entries
    set_distances(dist,inst);
	    
	//write tsp file    
	create_TSP_file(dist);
	//solve it
	int ret = 0;
    if(not use_LKH_)
        ret = system( (solver_call+" "+tsp_file+" &> /dev/null").c_str() );
    else
        ret = system( ("../LKH-2.0.6/test/LKH "+tsp_file+" "+tsp_sol_file).c_str());
        //TODO: better choice but results in segfault in LKH code!
        //solve_tsp(tsp_file,tsp_sol_file);

	if(ret!=0){
	    cerr<<"WARNING: Could not solve this file "<<tsp_file<<"!"<< endl;
	    return make_tuple(0,Tours{1});
	}    
		  
	//read the 	  
	vector<int> tsp_tour =  read_solution();	
    double bound = calculate_bound(tsp_tour,dist)/K;
    if(no_solution)
        return make_tuple(bound,Tours{K});
		  
    //read solution
    vector<vector<int>> vec = parse_solution(tsp_tour);	
    
    //delete everything
	if( remove( tsp_file.c_str() ) != 0 and verbosity_>0)
	    cerr<<"WARNING: Could not delete"<<tsp_file<<"!"<< endl;
	if( remove( tsp_sol_file.c_str() ) != 0 and verbosity_>0)
	    cerr<<"WARNING: Could not delete"<<tsp_sol_file<<"!"<< endl;
    if( system( "rm *TSP_temp_file*  &> /dev/null" )!=0 and verbosity_>0)
	    cerr<<"WARNING: Could not delete TMP files!"<< endl;  
    
	//DEBUG: INFO
	/*
	for(auto v:vec){
	    cout<<"t: ";
	    for(auto i: v)
            cout<<i<<" ";
        cout<<endl;
	}
	*/  

    //Find solution using the tsp solution as insertion ans assigment order
	InsertionHeuristic heur{local_search_};
	vector<uint> sequence;
	vector<uint> assignment;
	for(uint i=0;i<vec.size();++i)
	    for(uint j=0;j < vec[i].size();++j){
	        sequence.push_back(static_cast<uint>(vec[i][j]));
	        assignment.push_back(i);
	    }
	/*
	cout<<"sequence"<<endl;    
	for(auto e: sequence)
	    cout<<e<<" ";
	cout<<endl;   
	cout<<"\nassignment"<<endl;    
	for(auto e: assignment)
	    cout<<e<<" ";
	cout<<endl; 
	*/
	
    auto t = heur(inst, sequence, assignment);
    
	assert(inst.verify(t));
	return make_tuple(bound,t);
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
    file << "EDGE_WEIGHT_FORMAT :  FULL_MATRIX" <<endl;
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
    every job is split up into three parts.
        - alpha, somewhere between and beta
        - vertices: depot1, depot1, depot2, .... depotk, depotk,
                    j1, j1, j1, j2,....,jk, jk, jk, d12, d23, d34,...,dk-1k,dk1
        => job i at position i, 3i+1, 3i+2
        => index j belongs to job j/3, 
        - middle is only connected to alpha and beta
        - only alpha, beta connections
    */
    //lambdas for convenient access to the indice of jobs/depots    
    //numbers of jobs/depots are ranged from 0 to n-1/k-1
    
    auto setDist = [&](int i,int j,int d ){dist[i][j]=dist[j][i]=d;};
    
	//set selfloops to 0
	for(uint i=0; i< dist.size(); ++i)
	    dist[i][i]=0;
	    
	//connect all middle jobs to left and right 
	for(uint i=0; i<N;++i){
	    setDist(job_start(i),job_middle(i),0);
	    setDist(job_middle(i), job_end(i),inst[i].length());
	}
	//connect depot start and end (shortcuts tour, ie. no jobs in this tour) 
	for(uint i=0; i<K;++i)
	    setDist(depot_start(i),depot_end(i),0);   

    //connection between depots, cyclic edges with cost 0
    //use middle depot vertices
    for(uint i=0; i<K;++i){
	    setDist(depot_end(i),depot_middle(i),0);
	    setDist(depot_middle(i),depot_start((i+1)%K),0);	       
	}    
	    
	//connection between jobs (end<->start)    
	for(uint i=0; i<N;++i){
        for(uint j=i+1; j<N;++j){
	        auto& job_i = inst[i];
	        auto& job_j = inst[j]; 
	        setDist(job_start(i),job_end(j), dist_inf(job_i.alpha(),job_j.beta()));
	        setDist(job_end(i),job_start(j), dist_inf(job_i.beta(),job_j.alpha()));
	    }
	}

    //distance to/from depot, (start<->start, end<->end)
    for(uint i=0; i<N;++i){
        auto& job = inst[i];
        for(uint j=0; j<K;++j){
            auto d = dist_inf(inst.get_depot(j), job.alpha());
            setDist(depot_start(j),job_start(i),d);
            
            d = dist_inf(job.beta(), inst.get_depot(j));
            setDist(job_end(i),depot_end(j),d);
	    }    
    }        
}

std::vector<int> SingleCraneTSP_Solver::read_solution() const{
    fstream f;
    f.open(  tsp_sol_file.c_str(), ios::in );
    vector<int> numbers;

    //parse it
    if(not use_LKH_){
        string line, lines;
        getline(f, line);  // line 1
        while(getline(f, line))  //line 2...
            lines+=line;  
        vector<string> vec =  split(lines,' ');
        for(auto i: vec)
            numbers.push_back(stoi(i));    
    }else{
        string line;
        while(line!="TOUR_SECTION")
            getline(f, line);
        
        while(getline(f, line)){  //line after tour section.
            if(line == "-1" or line =="EOF") 
                break;
            numbers.push_back(stoi(line)-1);
        } 
    }

    f.close();
    return numbers;
}
	
double SingleCraneTSP_Solver::calculate_bound(const std::vector<int>& tour,
                               const std::vector<std::vector<int>> &dist) const
{
    double tour_length = 0;
    for(uint i=1;i<tour.size();++i)                    
        tour_length += dist[tour[i-1]][tour[i]];
    tour_length += dist[tour[tour.size()-1]][tour[0]];   
    return tour_length;                 
}
	
	    
vector<vector<int>> SingleCraneTSP_Solver::parse_solution(vector<int> &numbers) const{
    auto is_job       =  [this](uint i){return i>=2*K;};
    auto is_job_start =  [&,this](int i){return is_job(i) and (i-2*K)%3==0;};
    auto job_index    =  [this](int i){return (i-2*K)/3;}; 
    
    auto is_depot       =  [this](uint i){return i<2*K or i>=2*K+3*N;};
    auto is_depot_start =  [&,this](int i){return is_depot(i) and i%2==0;};
    auto is_depot_middle=  [this](uint i){return i>=2*K+3*N;};
    auto is_depot_end   =  [&,this](int i){return is_depot(i) and i%2==1;};
    auto depot_index    =  [this](int i){return i/2;}; 

  
    assert(numbers[0]==0);
    //algo: split in parts: split iff: interdepot edge
    //find first split edge!
    //insert correctly
    vector<vector<int>> paths;    
    for(uint i=0; i<K;++i)
        paths.push_back(std::vector<int>{});   
       
    uint M = numbers.size();
    assert(M==3*N+2*K+K);

    //Reverse if after the start of depot 0 comes an intermediate depot vertex  
    if (is_depot_middle(numbers[1])){
        reverse(begin(numbers)+1,end(numbers));
    }

    //split into subtours
    vector<int> *current = nullptr;
    for(uint i=0; i<M;++i){
        if(current==nullptr){
            if(is_depot_middle(numbers[i]))
                continue;
            if(not is_depot_start(numbers[i]))
                cout<< i <<": "<<numbers[i]<<endl;
            assert(is_depot_start(numbers[i]));
            current = &paths[depot_index(numbers[i])];
        }
        current->push_back(numbers[i]);
        if(is_depot_end(numbers[i])){
            current = nullptr;
        }
    }
     
    if(verbosity_ >= 1){
        cout<<"found paths"<<endl;
        for(uint i=0; i<paths.size();++i){
            cout<<i<<":  ";        
            for(auto e:paths[i]){
                cout<< e<<" ";
            }
            cout<<endl;
        }
    }
    
    //find a tour for every depot, skip tours visiting multiple depots
    vector<vector<int>> tours;
    vector<bool> depot_visited;
    for(uint i=0; i<K;i++) {
        tours.push_back(vector<int>{});
        depot_visited.push_back(false);
    }    
    //add tours
    for(uint i=0;i<K;++i){
        //already on other tour?
        if(depot_visited[i]) continue;
        //starting new and length == 2 => nop tour, just connection
        if(paths[i].size()==2) continue;
        //build tour, starting at depot i    
        int curdepot = i; 
        while(true){      
            depot_visited[curdepot]=true;
            for(uint j = 1; j<paths[curdepot].size()-1;++j)
                if(is_job_start(paths[curdepot][j]))
                    tours[curdepot].push_back(job_index(paths[curdepot][j]));
            if(paths[curdepot].back()==static_cast<int>(2*i+1)) break;
            curdepot = depot_index(paths[curdepot].back());   
        }
    }
    
    if(verbosity_ >=1 ){
        for(auto &t: tours){
            cout<<"";
            for(auto &e: t)
               cout<< e << " ";
            cout<<endl;       
        }
    }

    return tours;
}	    
	  
void SingleCraneTSP_Solver::build_tour(Tours &t, const vector<int> &vec,
                                       const Instance& inst, int vehicle) const
{
    double time = 0;
    auto pos = inst.get_depot(vehicle);
    for(uint i=0; i<vec.size();++i){
		int job = vec[i];
		time += dist_inf(pos,inst[job].alpha());
		t.add_job(&inst[job], time, vehicle);
		time += inst[job].length();
		pos = inst[job].beta();
    }
}	    

