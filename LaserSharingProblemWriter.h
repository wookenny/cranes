#pragma once

#include "Instance.h"
#include <string>
#include <zlib.h>

class ofstream;

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
**/

class LaserSharingProblemWriter{

	

	public:
	
		static constexpr char* DEFAULT_SUFFIX = ".xml";
		
		LaserSharingProblemWriter() = default;
		
		bool write(const Instance &i, std::string filename, bool zipped = false) const;
		
	private:
		std::string add_suffix(std::string s, std::string suffix) const;
		
		bool write_file_unzipped(const Instance &i, std::ofstream &out) const;
		bool write_file_zipped(const Instance &i, gzFile &outfile) const;
};
