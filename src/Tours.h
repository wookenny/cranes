#pragma once

/*
 * This file is part of a solving framework for 2DVS instances.
 * Copyright (C) 2013 Torsten Gellert <gellert@math.tu-berlin.de>
 */
#include <tuple>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <map>
#include <cassert>
#include <string>


class Job;

typedef std::tuple<const Job*, double> scheduledJob;

/*
* This class is used to build a solution for an instance.
* Further check for feasibility and the value of a solution is provided by the class Instance.
* Each tour keeps track of all inserted jobs and their starting times. 
* For each job the corresponding vehicle is known.
* 
* It can happen that two jobs with the same index are added. Nonetheless, a pointer
* can not occur twice during insertion of jobs. Index handling has to be done outside this class.
*/
class Tours{
private: 
    std::vector<std::vector<scheduledJob> > tours_;
    std::unordered_set<const Job*> _job_map;
    
    void _sort(int i);
    
    std::string to_string(unsigned int i) const;
    
    std::vector<scheduledJob> all_jobs() const;
    
public:
    Tours() = delete;
    /*Constructs k empty tours*/
    explicit Tours(uint k):tours_(k,std::vector<scheduledJob>()){}
    
    unsigned int num_tours() const{return tours_.size();}
    unsigned int num_jobs() const{return _job_map.size();}
    unsigned int num_jobs(unsigned int i) const{return tours_[i].size();}
    
    bool empty() const noexcept{ return _job_map.empty(); }
    
    bool contains(const Job* const job){return _job_map.find(job)!=_job_map.end();}
    
    
    const std::vector<scheduledJob>& operator[](int i) const{
        assert(0<=i and i < static_cast<int>(tours_.size()) );
        return tours_[i];
    }
    
    void clear(){ 
        _job_map.clear();
        for(auto& t: tours_) t.clear(); 
    }
    
    std::map<int,std::tuple<Job, double, int>> get_schedule() const;

    void add_job(const Job* const job, double time, int vehicle);

    void add_job(const scheduledJob& job, int vehicle) {
        add_job(std::get<0>(job), std::get<1>(job), vehicle);
    }

    void sort_jobs();

    std::string to_string() const;

    std::vector<uint> startingtime_permutation() const;

    std::vector<uint> endingtime_permutation() const;
};

/** Stream operator for convenience. 
Prints the string representation of tours. **/
inline
std::ostream& operator <<(std::ostream &os, const Tours &t) {
    os << t.to_string();
    return os;
}
