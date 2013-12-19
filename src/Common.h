#pragma once

#include <chrono>
#include <string>

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

