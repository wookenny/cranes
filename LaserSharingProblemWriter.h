#pragma once

#include "Instance.h"
#include <string>

#include <boost/iostreams/filtering_stream.hpp>


/**
The class LaserSharingProblemWriter is capable of writing instances of the 
2D Vehicle Scheduling Problem(2DVS) as a Laser Sharing Problem(lsp).

Since there are slight but significant differences, this reformulation is 
only adviced for problems of a certain type.
The 2DVS problem should hhave only drive-by instances, no edge should be too long 
and every pair of overlapping/touching edges counts as a forbidden pair.

The whole structure is modeled as a grid with 8 edges for each vertex with a 
travel time of 1 to each
other point. 


Big disadvantage: not allowed to stay on the same y-coordinate => worse solution 
if the x-line is too short
**/

class LaserSharingProblemWriter{

	

	public:
	
		static constexpr char* DEFAULT_SUFFIX = ".xml";
		static constexpr int SMALL_GRID_SIZE = 100;
		
		LaserSharingProblemWriter() = default;
		
		bool write(const Instance &i, std::string filename, bool zipped = false) const;
		
	private:
		std::string add_suffix(std::string s, std::string suffix) const;
		
		bool write_file(const Instance &i, boost::iostreams::filtering_ostream &out) const;
		bool check_instance(const Instance &i,bool verbose=false) const;
};
