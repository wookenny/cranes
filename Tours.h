#pragma once

#include <tuple>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <cassert>

class Job;

typedef std::tuple<const Job*, double> scheduledJob;

/*
This class is used to build a solution for an instance.
Further check for feasibility and the value of a solution is provided by the class instance.
Each tour keeps track of all inserted jobs and their starting times. 
For each job the corresponding vehicle is known.

It can happen that two jobs with the same index are added. Nonetheless, a pointer
can not occur twice during insertion of jobs. Index handling has to be done outside this class.
*/
class Tours{
	private: 
		std::vector<std::vector<scheduledJob> > _tours;
		std::unordered_set<const Job*> _job_map;
		
		void _sort(int i){  
			std::sort(_tours[i].begin(), _tours[i].end(), [](const scheduledJob &a,const scheduledJob &b){
      			return std::get<1>(a) < std::get<1>(b) or 
      				(  std::get<1>(a) == std::get<1>(b) and 
      				  std::get<0>(a)->length() < std::get<0>(b)->length());
      				//same starting time -> job with length 0 first
    		});
    	}
		
		std::string to_string(unsigned int i) const{
			std::string s;
			for(const scheduledJob &sj: _tours[i])
				s+= std::get<0>(sj)->to_string() +" @ "+std::to_string(std::get<1>(sj))+"\n";
			return s;
		}
		
				
	public:
		Tours() = delete;
		/*Constructs k empty tours*/
		Tours(int k):_tours(k,std::vector<scheduledJob>()){}
		//TODO: Do I need a move-constr./assign.?
		
		unsigned int num_tours() const{return _tours.size();}
		unsigned int num_jobs() const{return _job_map.size();}
		unsigned int num_jobs(unsigned int i) const{return _tours[i].size();}
		
		const std::vector<scheduledJob>& operator[](int i) const{ return _tours[i];}
		
		
		bool contains(const Job* const job){return _job_map.find(job)!=_job_map.end();}
		
		
		void add_job(const Job* const job, double time, int vehicle){ 
				assert(vehicle>=0);
				assert(vehicle < static_cast<int>( _tours.size()) ); 
				_tours[vehicle].push_back( std::make_tuple(job,time) );
				assert(!contains(job)); 
				_job_map.insert(job); 
        }
				

		void sort_jobs(){for(unsigned int i=0; i<_tours.size(); ++i) _sort(i);}
		
		std::string to_string() const{
			std::string s;
			for(unsigned int i=0; i<_tours.size();++i){
				s+=std::string("Tour ")+std::to_string(i)+":\n";
				s+=to_string(i);
			}	
			return s;	
		}

};

/** Stream operator for convenience. Prints the string representation of tours. **/
inline 
std::ostream& operator <<(std::ostream &os,const Tours &t)
{
	os<<t.to_string();
	return os;
}
