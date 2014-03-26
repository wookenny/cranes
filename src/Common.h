#pragma once

#include <string>
#include <chrono>
#include <vector>

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
    std::string dur;
    auto h = std::chrono::duration_cast<std::chrono::hours>(dtn).count();
    if (h > 0)
        dur += std::to_string(h)+" hours";
    auto m = std::chrono::duration_cast<std::chrono::minutes>(dtn).count();
    m -= h*60;
    if (m > 0)
        dur += (h > 0 ?" ":"")+std::to_string(m)+" minutes";
    auto s = std::chrono::duration_cast<std::chrono::seconds>(dtn).count();
    s -= (h*60*60 + m*60);
    if (s > 0)
        dur += ( (h > 0 or m > 0) ? " " : "")+std::to_string(s)+" seconds";

    return dur;
}

// used to build intervals for the batch mode
// exampels:
// * "a-d"   -> {a,b,c,d}
// * "[a,d]" -> {a,b,c,d}
// * "a"     -> {a}
// * "a,b,d" -> {a,b,d}
std::vector<std::string> create_interval(const std::string&);

// used to build find files recursively
// E.g.: "./f*/g*.png" find all files with name g*.png
// in folders starting with f in the current directory
std::vector<std::string> find_files(const std::string&);
