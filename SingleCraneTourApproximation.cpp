#include "SingleCraneTourApproximation.h"

#include <algorithm>
#include <vector>
#include <tuple>
#include <map>
#include <set>
#include <unordered_set>
#include <list>
#include <functional>

#include "PerfectMatching.h"
#include "Tours.h"
#include "Job.h"
#include "DisjointSet.h"

using namespace std;

class IncreasingIntervalMapping{
	private:
		map<int,int> forward;
		map<int,int> backward;
		
	public:
		IncreasingIntervalMapping() = default;
		int add(int i){
					assert(forward.find(i)==forward.end()); 
					int new_index = forward.size();
					forward[i] = new_index;
					backward[new_index] = i;
					return new_index;
					}
	    int new_name(int i)const {assert(forward.find(i)!=forward.end()); return forward.at(i);}
	    int old_name(int i) const {assert(backward.find(i)!=backward.end());return backward.at(i);}
	    string to_string() const{
	    	string s("Mapping:\n");
	    	for(const auto &m: forward)
	    		s += std::to_string(m.first)+" -> "+std::to_string(m.second)+"\n";
	    	return s;
	    }
};
inline 
std::ostream& operator <<(std::ostream &os,const IncreasingIntervalMapping &map)
{
	os<< map.to_string();
	return os;
}


//forward declaration of all used simple helper functions
vector<int> perfect_matching(const Instance &inst);
multimap<int,int> calculate_mst_edges();
void calculate_startingtimes(Tours &tour, const Instance &inst, 
									const vector<vector<int>> &ordered_jobsets );				
multimap<int,int> calculate_mst_edges(const Instance &inst, const vector<int> &next);							
multimap<int,int> contracting_mst_edges(const Instance &inst);
multimap<int,int> odd_degree_matching(const Instance &inst, multimap<int,int> &mst_edges);
vector<vector<int>>  build_tours_from_edges(const Instance &inst, 
												multimap<int,int> &mst_edges, 
												vector<int> &matching);	
vector<vector<int>>  build_tours_from_edges(const Instance &inst, 
												multimap<int,int> &mst_edges, 
												multimap<int,int> &matching);		


Tours SingleCraneTourApproximation::operator()(const Instance& inst) const{
	//calculate largeedge tour and large arc tour and return the shorter one
	Tours t1  	= large_arc_(inst);
	
	Tours t2 	= large_edge_(inst);
	
	return inst.makespan(t1) < inst.makespan(t2) ? t1 : t2; 

}


Tours SingleCraneTourApproximation::large_arc_(const Instance& inst) const{

	vector<int> matching = perfect_matching(inst);
	multimap<int,int> mst_edges = calculate_mst_edges(inst,matching);

	auto ordered_jobsets = build_tours_from_edges(inst,mst_edges, matching);	
	//TODO: reverse the orientation and take the better one	
	Tours tour(inst.num_vehicles());
	calculate_startingtimes(tour,inst,ordered_jobsets);
	
	return tour;
}


Tours SingleCraneTourApproximation::large_edge_(const Instance& inst) const{
	
	
	multimap<int,int> mst_edges = contracting_mst_edges(inst);
	
	//TODO: Debug
	//cout<< "instance:\n"<<inst << endl;
	//cout << "Found MST edges:"<< endl;
	//for(auto &edge: mst_edges)
	//	cout<< edge.first+1 << "->" <<edge.second+1 <<endl;
	//cout<<endl;
	
	multimap<int,int> matching_edges = odd_degree_matching(inst, mst_edges);
	//TODO: Debug
	//cout << "Found additional matching edges:"<< endl;
	//for(auto &edge:matching_edges)
	//	cout<< edge.first+1 << "->" <<edge.second+1 <<endl;
	//cout<<endl;
	
	
	auto ordered_jobsets = build_tours_from_edges(inst,mst_edges, matching_edges);		
	//TODO:DEBUG
	//cout<< "Found assignment:"<<endl;
	//for(auto tour: ordered_jobsets){
	//	cout<<"- "; 
	//	for(auto t:tour)
	//		cout<<t<<" ";
	//	cout<<endl;
	//}					
	Tours tour(inst.num_vehicles());
	calculate_startingtimes(tour,inst,ordered_jobsets);
	
	return tour;	
}



/**
* Only the helper functions from here on. 
*/

void calculate_startingtimes( Tours &tour, const Instance &inst, 
									const vector<vector<int>> &ordered_jobsets ){
	//iterate over all tours and set correct increasing starting times
	for(uint i=0; i<inst.num_vehicles(); ++i){
		int time = 0;
		auto position = inst.get_depot(i);
		//iterate ove all visited jobs and increment time:
		for(int job: ordered_jobsets[i]){
			if(job > static_cast<int>(inst.num_jobs())) continue;
			time += dist_inf(position,inst[job].alpha());
			tour.add_job(&inst[job], time, i);
			time += inst[job].length();
			position = inst[job].beta();
		}
	}

}

vector<int> perfect_matching(const Instance &inst){

	const int N = inst.num_jobs();
	const int K = inst.num_vehicles();
	
	int node_num = 2*inst.num_jobs();
	vector<int> next(inst.num_jobs(), -1);

	//find min cost perfect matching(between start end end of jobpoints)	
	PerfectMatching pm(node_num, inst.num_jobs() * inst.num_jobs());
	pm.options.verbose = false;
	//for each job pair an edge in both directions
	for(uint i = 0; i < inst.num_jobs() ; ++i)
		for(uint j = 0; j < inst.num_jobs() ; ++j){
			const Job& job1 = inst[i]; 
			const Job& job2 = inst[j]; 
			double weight = dist_inf(job1.get_beta(), job2.get_alpha());
			pm.AddEdge(2*i,2*j+1,weight);	
		}

	pm.Solve();
	//increment with 2, becase all even numbers correspond to an end of a job
	for(int i=0; i< node_num; i+=2 ){
		next[i/2] = (pm.GetMatch(i)-1)/2;
	}
	
	//here we add k depots. The are connected as a single circle
	for(int i=0; i<K; ++i)
		next.push_back( N + (i+1)%K );	
		
	return next;	
}


multimap<int,int> calculate_mst_edges(const Instance &inst, const vector<int> &next){
	const int N = inst.num_jobs();
	const int K = inst.num_vehicles();
	
	//build datastructure for information about disjoint sets
	DisjointSet disjoint_sets( next );
	
	//create all possible edges
	typedef tuple<int,int,int> WeightedEdge;
	vector<WeightedEdge> all_edges;
	for(int i = 0; i < N  ; ++i)
		for(int j = i+1; j < N+K ; ++j){ //no inter-depot connections needed
				if( j<N ){//j is a proper job
					const Job& job1 = inst[i]; 
					const Job& job2 = inst[j]; 
					int min_dist = dist_inf(job1.get_alpha(), job2.get_alpha());
					min_dist = min(min_dist,dist_inf(job1.get_alpha(), job2.get_beta()) );
					min_dist = min(min_dist,dist_inf(job1.get_beta(), job2.get_alpha()) );
					min_dist = min(min_dist,dist_inf(job1.get_beta(), job2.get_beta()) );
					all_edges.push_back( make_tuple(i,j,min_dist) );
				}else{//j is a depot
					auto depot_pos = inst.get_depot(j-N); 
					const Job& job1 = inst[i]; 
					int min_dist = dist_inf(depot_pos, job1.get_alpha());
					min_dist = min(min_dist,dist_inf(depot_pos, job1.get_beta()) );
					all_edges.push_back( make_tuple(i,j,min_dist) );
				}
			}
	
	//sort the edges due to their length/cost	
	auto sorter = [] (const WeightedEdge& u, 
					  const WeightedEdge& v) -> bool { return get<2>(u) < get<2>(v); };
  
	sort(all_edges.begin(), all_edges.begin() + all_edges.size(), sorter);
		  	
	//add edges until the graph is completly connected
	multimap<int,int> mst_edges;
	int current_edge = 0;
	while(disjoint_sets.size()>1){
		assert(static_cast<int>(all_edges.size()) > current_edge);
		int u = get<0>( all_edges[current_edge] );
		int v = get<1>( all_edges[current_edge] );
		if(disjoint_sets.findSet(u) != disjoint_sets.findSet(v)){
			disjoint_sets.unionSets(u,v);
			//edges in both directions added
			mst_edges.insert(pair<int,int>(u,v));
			mst_edges.insert(pair<int,int>(v,u));
		}
		++current_edge; 
	}

	return mst_edges;
}


vector<vector<int>>  build_tours_from_edges(const Instance &inst, 
												multimap<int,int> &mst_edges, 
												vector<int> &matching){
	
	const int N = inst.num_jobs();
	const int K = inst.num_vehicles();
	
	set<pair<int,int>> traversed_mst_edges;
	vector<vector<int>> ordered_jobsets(K,vector<int>());
		
	for(int vehicle=0; vehicle<K;++vehicle){
		int current_job = N+vehicle;	
			
		//stop this loop if N is the starting depot and no 
		// other MST edge available	
		while(  N+vehicle != current_job or 
				mst_edges.find(current_job)!=mst_edges.end()){

			//update position if not a depot
			if(current_job < N){
				const auto& vec = ordered_jobsets[vehicle];
				bool job_added = 
					find(vec.begin(), vec.end(), current_job)!= vec.end();
					
				if(not job_added)
					ordered_jobsets[vehicle].push_back(current_job);
			}
			
			
			//any non-backward traversed edge?
			auto iter = mst_edges.find(current_job);
			bool traversed_backwards = true;
			while( iter!=mst_edges.end()){
				//cout<<"Iter wert: "<< iter->first+1<<" -> "<<iter->second+1 <<endl;
				//break if edge does not start at current_job
				if(iter->first!=current_job)
					break;
				
				//break if NOT traversed backwards, candidate found
				if (traversed_mst_edges.find(pair<int,int>(iter->second,iter->first))==
					 traversed_mst_edges.end() ){ 
					traversed_backwards = false; 
					break;	 
				}
				++iter;	
			} 
			
			if(traversed_backwards)
				iter = mst_edges.find(current_job);

   			
			//prioritize unseen MST edges
			if(iter!=mst_edges.end() and not traversed_backwards){
				traversed_mst_edges.insert(pair<int,int>(iter->first,iter->second));
				current_job = iter->second;
				
				mst_edges.erase(iter);
			}else if(matching[current_job]!=-1){	//use matching edge is no MST edge
				int next_job = matching[current_job];
				matching[current_job] = -1;
				current_job = next_job;
			}else{		
					assert( iter != mst_edges.end() );
					assert( iter->first == current_job );	
					current_job = iter->second;
					mst_edges.erase(iter);
			}
		}

	}
	return ordered_jobsets;
}


// large edge helper

multimap<int,int> contracting_mst_edges(const Instance &inst){
	
	const int N = inst.num_jobs();
	const int K = inst.num_vehicles();
	
	//build datastructure for information about disjoint sets
	DisjointSet disjoint_sets(N+K);
	
	//implicit connections between depots:
	for(int i=1; i<K;++i)
		disjoint_sets.unionSets(N,N+i);
	
	//create all possible edges
	typedef tuple<int,int,int> WeightedEdge;
	vector<WeightedEdge> all_edges;
	for(int i = 0; i < N  ; ++i)
		for(int j = i+1; j < N+K ; ++j){ //no inter-depot connections needed
				if( j<N ){//j is a proper job
					const Job& job1 = inst[i]; 
					const Job& job2 = inst[j]; 
					int min_dist = dist_inf(job1.get_alpha(), job2.get_alpha());
					min_dist = min(min_dist,dist_inf(job1.get_alpha(), job2.get_beta()) );
					min_dist = min(min_dist,dist_inf(job1.get_beta(), job2.get_alpha()) );
					min_dist = min(min_dist,dist_inf(job1.get_beta(), job2.get_beta()) );
					all_edges.push_back( make_tuple(i,j,min_dist) );
				}else{//j is a depot
					auto depot_pos = inst.get_depot(j-N); 
					const Job& job1 = inst[i]; 
					int min_dist = dist_inf(depot_pos, job1.get_alpha());
					min_dist = min(min_dist,dist_inf(depot_pos, job1.get_beta()) );
					all_edges.push_back( make_tuple(i,j,min_dist) );
				}
			}
	
	//sort the edges due to their length/cost	
	auto sorter = [] (const WeightedEdge& u, 
					  const WeightedEdge& v) -> bool { return get<2>(u) < get<2>(v); };
  
	sort(all_edges.begin(), all_edges.begin() + all_edges.size(), sorter);
	
	//add edges until connected. add each edge twice(-> both directions)
	//add edges until the graph is completly connected
	multimap<int,int> mst_edges;
	int current_edge = 0;
	while(disjoint_sets.size()>1){
		assert(static_cast<int>(all_edges.size()) > current_edge);
		int u = get<0>( all_edges[current_edge] );
		int v = get<1>( all_edges[current_edge] );
		if(disjoint_sets.findSet(u) != disjoint_sets.findSet(v)){
			disjoint_sets.unionSets(u,v);
			//edges in both directions added
			mst_edges.insert(pair<int,int>(u,v));
			mst_edges.insert(pair<int,int>(v,u));
		}
		++current_edge; 
	}
	
	//add implicit zero-cost edges
	/*
	for(int i=1; i<K;++i){
		mst_edges.insert(pair<int,int>(N,N+i));
		mst_edges.insert(pair<int,int>(N+i,N));
	}
	*/
	return mst_edges;
}

multimap<int,int> odd_degree_matching(const Instance &inst, multimap<int,int> &mst_edges){
	const int N = inst.num_jobs();
	const int K = inst.num_vehicles();
	
	multimap<int,int> matching_edges;
	
	//find nodes with odd degree -> count outgoing edges
	vector<int> odd_degree_jobs;
	for(int i=0; i<N+K;++i)
		if( 1 == mst_edges.count(i) %2 ){
			odd_degree_jobs.push_back(i);
			
		}
	
	if(odd_degree_jobs.empty()) return matching_edges;
	//find matching
	IncreasingIntervalMapping map;
	for(int i: odd_degree_jobs)
		map.add(i);
	//TODO: DEBUG
	//cout<< "Odd degree vertices:"<<endl;
	//for(int i: odd_degree_jobs)
	//	cout << i+1<<endl;
	//cout<<map<<endl;	
	
	int num_edges = odd_degree_jobs.size()*(odd_degree_jobs.size()-1);
	PerfectMatching pm( odd_degree_jobs.size(), num_edges );
	pm.options.verbose = false;
	//for each job pair an edge in both directions
	for(int i: odd_degree_jobs)
		for(int j: odd_degree_jobs){
			if(i==j) continue;
			int weight;
			if(i<N and j<N){//two jobs
				const Job& job1 = inst[i]; 
				const Job& job2 = inst[j]; 
				//get shortest of all 4 possibilites
				weight = dist_inf(job1.get_alpha(), job2.get_alpha());
				weight = min(weight, dist_inf(job1.get_alpha(), job2.get_beta()) );
				weight = min(weight, dist_inf(job1.get_beta(),  job2.get_alpha()) );
				weight = min(weight, dist_inf(job1.get_beta(),  job2.get_beta()) );				 
			}else if(i<N and j>=N){//job and depot
				const Job& job = inst[i];
				auto depot_position = inst.get_depot(j-N);
				//get shortest of both possibilites
				weight = dist_inf(job.get_alpha(), depot_position);
				weight = min(weight, dist_inf(job.get_beta(), depot_position) );
			}else if(i>=N and j<N){//depot and job
				const Job& job = inst[j];
				auto depot_position = inst.get_depot(i-N);
				//get shortest of both possibilites
				weight = dist_inf(job.get_alpha(), depot_position);
				weight = min(weight, dist_inf(job.get_beta(), depot_position));
			}else{//depot and depot
				auto depot1 = inst.get_depot(i-N);
				auto depot2 = inst.get_depot(j-N);
				weight = dist_inf(depot1, depot2);
			}
			//TODO: DEBUG
			//cout<< "add edge: "<<i+1<<" -> "<<j+1<<endl;
			pm.AddEdge(map.new_name(i), map.new_name(j), weight);	
		}

	pm.Solve();
	//parse matching, each machting is added twice(once for each adjacent node)
	for(int job: odd_degree_jobs){
		int matched = pm.GetMatch(map.new_name(job));
		matched = map.old_name(matched);
		matching_edges.insert( pair<int,int>(job,matched) );
	}
	return matching_edges;
}

vector<vector<int>>  build_tours_from_edges(const Instance &inst, 
												multimap<int,int> &mst_edges, 
												multimap<int,int> &matching){
											
	const int N = inst.num_jobs();
	const int K = inst.num_vehicles();
			
	vector<vector<int>> ordered_jobsets(K,vector<int>());
	//only one set of edges desired:
	mst_edges.insert(matching.begin(), matching.end());
	
	for(int vehicle=0; vehicle<K;++vehicle){
		int current_job = N+vehicle;	
	
		set<int> active_nodes;
		list<int> tour;
		tour.push_back(current_job);
		
		auto insertion_position = tour.begin(); ++insertion_position; 
		active_nodes.insert( *tour.begin() );
	 
	 	//cout<< "tour "<<vehicle<<endl;
		while( not active_nodes.empty()){
			//cout<<"current pos: "<<current_job<<endl;
			//go on with regular job, if there is a usable edge
			auto next_job =  mst_edges.find(current_job);
			if( next_job!=mst_edges.end() ){
				assert(current_job == next_job->first);
				int old_job = current_job;
				current_job = next_job->second;
				//cout<<"using edge to reach"<<current_job<<endl;
				mst_edges.erase(next_job);
				//erase anti-edge
				auto anti_edge= mst_edges.find( current_job);
				while(anti_edge!=mst_edges.end() and anti_edge->second != old_job)
					++anti_edge;
				assert(anti_edge!=mst_edges.end() and anti_edge->first == current_job);
				mst_edges.erase(anti_edge);
				
				//anorther outgoing edge? => addto active nodes
				if(mst_edges.find(old_job)!=mst_edges.end())
					active_nodes.insert(old_job); //if already contained, nothing happens
				else
					active_nodes.erase(old_job);
				
				tour.insert(insertion_position, current_job);	
				//++insertion_position;								
			}else{//or jump to another unfinished cycle.
				assert(not active_nodes.empty());
				current_job = *active_nodes.begin();
				//cout<< "adding cycle starting at "<<current_job<<endl;
				active_nodes.erase(active_nodes.begin());
				auto position = find(tour.begin(), tour.end(), current_job);
				assert(position!=tour.end());
				insertion_position = position; 
				++position;				
			}	
		}
		
		//cout<<"\ntour: "<<endl;
		set<int> added_jobs;
		for(auto job: tour){
			if(job >= N) continue;  
			if(added_jobs.find(job)!=added_jobs.end()){
				//cout<<"shortcutting "<<job <<endl;
				continue;
			}
			ordered_jobsets[vehicle].push_back(job);
			added_jobs.insert(job);
			//cout<< job<<" ";
		}
		//cout<<endl;
		//ordered_jobsets[vehicle].insert(ordered_jobsets[vehicle].begin(),tour.begin(),tour.end());
	}
	
	return ordered_jobsets;
												
}
