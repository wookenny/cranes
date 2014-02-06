#pragma once

#include <string>
#include <chrono>
#include <vector> 

std::vector<std::string> &split(const std::string &s, char delim, 
                           std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);


void replaceAll(std::string& str, const std::string& from, const std::string& to);

std::tuple<std::string,std::string> fixed_prefix_path(const std::string &s);

//split string at every whitespace position
std::vector<std::string> split(const std::string &input);


//cyclic slice of vector vec, including i, exluding j
/*
template <typename T>
std::vector<T> slice(std::vector<T> vec, size_t i,size_t j){
    std::vector<T> result;
    assert(0>=i and 0>=j and i <= vec.size() and j <= vec.size());
    for(size_t p=i; i!=j; i=(i+1)%vec.size())    
        result.push_back(vec[p]);
    return result;    
}
*/

template<class Duration>
std::string duration_to_string(const Duration& dtn){
    std::string dur;
    auto h = std::chrono::duration_cast<std::chrono::hours>(dtn).count();
    if(h>0)
        dur += std::to_string(h)+" hours";
    auto m = std::chrono::duration_cast<std::chrono::minutes>(dtn).count();
    m -= h*60;
    if(m>0)
        dur += (h>0?" ":"")+std::to_string(m)+" minutes";
    auto s = std::chrono::duration_cast<std::chrono::seconds>(dtn).count();
    s -= (h*60*60 + m*60);
    if(s>0)
        dur += ((h>0 or m > 0)?" ":"")+std::to_string(s)+" seconds";    
            
    return dur;
}

//used to build intervals
std::vector<std::string> create_interval(const std::string& );
//used to build intervals
std::vector<std::string> find_files(const std::string& );
