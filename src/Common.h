#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <functional>
#include <algorithm>
#include <limits>

template <typename Iterator>
bool next_combination(const Iterator first, Iterator k, const Iterator last){
    /* Credits: Mark Nelson http://marknelson.us */
    if ((first == last) || (first == k) || (last == k))
        return false;
    Iterator i1 = first;
    Iterator i2 = last;
    ++i1;
    if (last == i1)
        return false;
    i1 = last;
    --i1;
    i1 = k;
    --i2;
    while (first != i1){
        if (*--i1 < *i2){
            Iterator j = k;
            while (!(*i1 < *j)) ++j;
            std::iter_swap(i1,j);
            ++i1;
            ++j;
            i2 = k;
            std::rotate(i1,j,last);
            while (last != j){
                ++j;
                ++i2;
            }
            std::rotate(k,i2,last);
            return true;
        }
    }
    std::rotate(first,k,last);
    return false;
}
//usage is in the GTests!



std::vector<uint> random_assignment(uint size, uint lb, uint ub, int seed=0);

std::vector<std::string> split(const std::string &s, char delim);

// replaces all occureances of from in str with to
void replaceAll(std::string& str, const std::string& from,
                                  const std::string& to);

// split string at every whitespace position
std::vector<std::string> split(const std::string &input);


// convert a duration object into a nice and readable string, represeting
// the same duration
template<class Duration>
std::string duration_to_string(const Duration& dtn) {
    if(std::chrono::duration_cast<std::chrono::seconds>(dtn).count() == 0)
        return "0 seconds";

    std::string dur;
    auto h = std::chrono::duration_cast<std::chrono::hours>(dtn).count();
    if (h > 0)
        dur += std::to_string(h)+(h==1?" hour":" hours");
    auto m = std::chrono::duration_cast<std::chrono::minutes>(dtn).count();
    m -= h*60;
    if (m > 0)
        dur += (h > 0 ?" ":"")+std::to_string(m)+(m==1?" minute":" minutes");
    auto s = std::chrono::duration_cast<std::chrono::seconds>(dtn).count();
    s -= (h*60*60 + m*60);
    if (s > 0)
        dur += ( (h > 0 or m > 0) ? " " : "")+std::to_string(s)+(s==1?" second":" seconds");

    return dur;
}


inline bool logically_equal(double a, double b, double error_factor=1.0)
{
  return a==b or
    std::abs(a-b) < std::abs(std::min(a,b)) *
                    std::numeric_limits<double>::epsilon() *
                    error_factor;
}

std::string minutes_to_string(double time);


// used to build intervals for the batch mode
// examples:
// * "a-d"   -> {a,b,c,d}
// * "[a,d]" -> {a,b,c,d}
// * "a"     -> {a}
// * "a,b,d" -> {a,b,d}
// HINT: this workls only for numbers!
// non numbers: "s,t,q" -> {s,t,q}
std::vector<std::string> create_interval(const std::string&);

// used to build find files recursively
// E.g.: "./f*/g*.png" find all files with name g*.png
// in folders starting with f in the current directory
std::vector<std::string> find_files(const std::string&);

bool is_permutation(std::vector<uint> v);

std::vector<uint> random_permutation(uint n, int seed = -1);

std::function<bool ()> getTimer(int timelimit); 

std::string to_str(const std::vector<std::string>& vec, bool braces=false);

std::string itos(int value, int base);