#include "./Tours.h"

#include "./Job.h"
#include "./Common.h"

void Tours::_sort(int i){  
	std::sort(tours_[i].begin(), tours_[i].end(), 
				[](const scheduledJob &a,const scheduledJob &b){
                    //same starting time(doublevalue) -> job with length 0 first
                    if( logically_equal(std::get<1>(a),std::get<1>(b),10) )
                        return (std::get<0>(a)->length() < std::get<0>(b)->length());
                    //rearlierjob first
        			return (std::get<1>(a) < std::get<1>(b)); 
       			}
       		);
}

std::string Tours::to_string(unsigned int i) const{
	std::string s;
	for(const scheduledJob &sj: tours_[i]){
		s+= std::get<0>(sj)->to_string();
		s+=" @ "+std::to_string(std::get<1>(sj))+"\n";
	}
	return s;
}

std::map<int,std::tuple<Job, double, int>> Tours::get_schedule() const {
    std::map<int,std::tuple<Job, double, int>> schedule;
    //ID -> Job, starting time, vehicle
    for(uint i=0; i<tours_.size();++i)
            for(const auto job: tours_[i]){
            Job j =  *std::get<0>(job);
                double time =  std::get<1>(job);
           schedule[j.num()] = std::make_tuple(j,time,i);
       }
    return schedule;
}

void Tours::add_job(const Job* const job, double time, int vehicle) {
    assert(vehicle >= 0);
    assert(vehicle < static_cast<int>(tours_.size()));
    assert(not contains(job));
    tours_[vehicle].push_back(std::make_tuple(job, time));
    _job_map.insert(job);
}

std::string Tours::to_string() const{
    std::string s;
    for (unsigned int i = 0; i < tours_.size(); ++i) {
        s+=std::string("Tour ")+std::to_string(i)+":\n";
        s+=to_string(i);
    }
    return s;
}

void Tours::sort_jobs(){
    for(unsigned int i = 0; i < tours_.size(); ++i) 
        _sort(i);
}

std::vector<scheduledJob> Tours::all_jobs() const {
    std::vector<scheduledJob> jobs;
    for(auto& t: tours_){
        jobs.insert(std::end(jobs),std::begin(t),std::end(t));
    }
    return jobs;
}

std::vector<uint> Tours::startingtime_permutation() const{
    std::vector<uint> perm;
    
    auto jobs = all_jobs();
    std::sort(begin(jobs), end(jobs), 
                [](const scheduledJob &a,const scheduledJob &b){
                    return std::get<1>(a) < std::get<1>(b);
                }
            );
    for(auto j: jobs)
        perm.push_back(std::get<0>(j)->num() - 1);

    assert( is_permutation(perm) );
    return perm;
}

std::vector<uint> Tours::endingtime_permutation() const{
    std::vector<uint> perm;

    auto jobs = all_jobs();
    std::sort(begin(jobs), end(jobs), 
                [](const scheduledJob &a,const scheduledJob &b){
                    return std::get<1>(a) + std::get<0>(a)->length()
                           < std::get<1>(b) + std::get<0>(b)->length();
                }
            );
    for(auto j: jobs)
        perm.push_back(std::get<0>(j)->num() - 1);

    assert( is_permutation(perm) );
    return perm;
}