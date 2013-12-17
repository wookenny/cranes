#include "SingleCraneTSP_Solver.h"
#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdlib>

using namespace std;


vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}


const string solver_call = "concorde"; 
const string tsp_file = "TMP_TSP_FILE.TSP";
const string tsp_sol_file = "TMP_TSP_FILE.sol";  

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
	int ret = system( (solver_call+" "+tsp_file+" > /dev/null").c_str() );
	if(ret!=0){
	    cerr<<"WARNING: Could not delete"<<tsp_file<<"!"<< endl;
	    return Tours{1};
	}    
		  
    //read solution
    vector<int> vec = parse_solution();	
	
	//delete everything
	if( remove( tsp_file.c_str() ) != 0 )
	    cerr<<"WARNING: Could not delete"<<tsp_file<<"!"<< endl;
	if( remove( tsp_sol_file.c_str() ) != 0 )
	    cerr<<"WARNING: Could not delete"<<tsp_sol_file<<"!"<< endl;
	if( system( "rm TMP_TSP_FILE.* OTMP_TSP_FILE.*" )!=0)
	    cerr<<"WARNING: Could not delete TMP files!"<< endl;

	    
	Tours t{1};
	build_tour(t,vec,inst);
	
	cout<< "found tour: "<<inst.makespan(t)<<endl; 
	cout<< "tour valid?: "<<inst.verify(t)<<endl;
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
	    
vector<int> SingleCraneTSP_Solver::parse_solution()const{
    fstream f;
    f.open(  tsp_sol_file.c_str(), ios::in );
    //parse it
    string line, lines;
    getline(f, line);  // have line 1
    while(getline(f, line))  //line 2...
        lines+=line;
    vector<string> vec =  split(lines,' ');
    vector<int> numbers;
    for(auto i: vec)
        numbers.push_back(stoi(i));

    int start = 0;
    while(numbers[start]!=0)
        ++start;      
       
    bool forward = (numbers[(start+1)%numbers.size()]==1);  
    auto modulus = [=](int a, int m) ->int {return (a+m)%m;};
    
    vector<int> tour;
    int pos = start + forward?1:-1;
    int m = numbers.size(); 
    pos = modulus(pos,m);

    for(int i=0; i<m/3;++i){
        assert(numbers[pos]/3 == numbers[modulus(pos+1,m)]/3);
        assert(numbers[pos]/3 == numbers[modulus(pos-1,m)]/3);
        tour.push_back(numbers[pos]/3);
        if(forward)
            pos+=3;
        else
            pos-=3;
        pos = modulus(pos,m);
    }
    f.close();
    
    return tour;
}	    
	  
void SingleCraneTSP_Solver::build_tour(Tours &t,const vector<int> &vec,
                                            const Instance& inst) const
{
    assert(vec[0]==0);
    //TODO: such a pice of code should be added to the tours class!
    auto position = inst.get_depot(0);
    double time = 0;
    for(uint i=1; i<vec.size();++i){
		int job = vec[i]-1;
		time += dist_inf(position,inst[job].alpha());
		t.add_job(&inst[job], time, 0);
		time += inst[job].length();
		position = inst[job].beta();
    }
    
}	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
	    
