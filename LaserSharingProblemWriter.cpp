#include "LaserSharingProblemWriter.h"

#include <iostream>
#include <iomanip>
#include <fstream>

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>

using namespace std;


bool LaserSharingProblemWriter::write(const Instance &i, std::string filename, bool zipped) const{
	filename = add_suffix(filename, DEFAULT_SUFFIX);

	namespace io = boost::iostreams; 
	boost::iostreams::filtering_ostream out; 
	
	if(zipped){
		out.push( io::gzip_compressor());
		filename = add_suffix(filename, ".gz");
	}
	out.push( io::file_sink(filename, ios_base::out | ios_base::binary)); 

	//write instance
	write_file(i,out);
	
	//end
	out<<"\n";
	out.flush(); 
	out.reset(); 

	return true;
}	

std::string LaserSharingProblemWriter::add_suffix(string s,string suffix) const{
	if (suffix[0] !='.')
		suffix = "."+suffix;
		
	//if string ends with suffix, do nothing
	if(s.size() > suffix.size() ){
		//check the ending of s
		if(s.substr(s.length()-suffix.size(),suffix.size()) == suffix)
			return s;
	}
		
	//else add the suffix
	return s+suffix;
}

/*TODO: Fragen an Wolfgang
 - at which copy of a grid point does a job start? 
 - it has to be reachable by all!
 - Problem: selbst mit zusatzraum knoten sind die instanzen nicht gleich,
 - selbst, wenn man alle jobs disjunkt starten läßt -> man bräuchte sowas wie: job
  startet aus einem beliebigen Knoten der zusatzraum knoten  
*/
bool LaserSharingProblemWriter::write_file(const Instance &inst, 
								boost::iostreams::filtering_ostream &out) const{
								
   int makespan = 9000; //or over 9000?								

	auto grid = [](int x,int y,int v){ return string("G_")+to_string(x)
					+"_"+to_string(y)+"_"+to_string(v); };	
	
								
	out<< "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>"
	  <<"<lsp cycletime=\""<< makespan <<"\""
	  <<" xmlns=\"http://www.wm.uni-bayreuth.de/fileadmin/Cornelius/XMLSchema/lsp"
	  <<"\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
	  <<" xsi:schemaLocation=\"http://www.wm.uni-bayreuth.de/fileadmin"
	  <<"/Cornelius/XMLSchema/lsp http://www.wm.uni-bayreuth.de/fileadmin"
	  <<"/Cornelius/XMLSchema/lsp.xsd\">\n\n";							
	
	//robots
	out << "  <robots>\n";
	for(uint i=1;i<=inst.num_vehicles();++i){
		out<<"    <robot name=\""<< i <<"\">\n";
		out<<"      <home name=\"";
		out<< grid(inst.get_depot(i)[0],inst.get_depot(i)[1]);
		out<<"\">\n";
		out<<"    </robot>\n";
	}
    out<<"\n";
    			
    //jobs			
    out << "  <jobs>\n";
	for(const auto &job: inst){
		out << "    <job direction=\"fixed\" name=\"J"<<job.num()<<"\">\n";
		out << "      <aPos name=\""<<grid(job.alpha()[0],job.alpha()[1])<<"\"/>\n";
		out << "      <bPos name=\""<<grid(job.beta()[0],job.beta()[1])<<"\"/>\n";
		out << "      <robots>\n";
		for(uint i=1;i<=inst.num_vehicles();++i)
			out << "        <robot name=\"" << i << "\"/>\n";
		out << "       </robots>\n";
    	out << "    <job>\n"; 		
    }
    out << "  </jobs>\n\n";

    
    //write distances 
     out << "  <distances>\n";
    //drive-by jobs
    
    auto add_dist = [&] (string orig, string dest, double dist) { 
    	for(uint i=1;i<=inst.num_vehicles();++i)
    	out << "    <dist from=\""<< orig << "\" robot=\""<<i
    		 <<"\" to=\""<< dest <<"\">"<< dist <<"</dist>"<<endl;
    };
    
    out<<fixed<<setprecision(6)<<fixed;
    for(const auto &job: inst)
    	add_dist("J"+to_string(job.num())+"A","J"+to_string(job.num())+"B", 0);

    //distances for the whole grid
    auto bbox = inst.get_bounding_box();
    int min_x, max_x, min_y, max_y;
    min_x = bbox[0]; min_y = bbox[1];
    max_x = bbox[2]; max_y = bbox[3];
    for(int x = min_x; x <= max_x;++x){
    	for(int y = min_y; y <= max_y; ++y){
    		//add up to eight links to neighboring vertices
    		add_dist(grid(x,y), grid(x,y), 0);	
    		if(x > min_x)
    			add_dist(grid(x,y), grid(x-1,y), 1);
    		if(x < max_x)
    			add_dist(grid(x,y), grid(x+1,y), 1);
    		if(y > min_y)
    			add_dist(grid(x,y), grid(x,y-1), 1);
    		if(y < max_y)
    			add_dist(grid(x,y), grid(x,y+1), 1);	
    		if(x > min_x and y > min_y)
    			add_dist(grid(x,y), grid(x-1,y-1), 1);
    		if(x > min_x and y < max_y)
    			add_dist(grid(x,y), grid(x-1,y+1), 1);
    		if(x < max_x and y > min_y)
    			add_dist(grid(x,y), grid(x+1,y-1), 1);
    		if(x < max_x and y < max_y)
    			add_dist(grid(x,y), grid(x+1,y+1), 1);	
    					
    	}
    }
    out << "  </distances>\n\n";
        
    auto coll_line = [&] (string orig, string dest, int r) {
    	return string("      <line from=\"")+orig+"\" robot=\""+
    			to_string(r)+"\" to=\""+dest+"\"/>\n";
    };    
        
    //write collisions
    out << "  <collisions>\n";
	//point collisions: all on the same y-coordinate
	for(int x = min_x; x <= max_x; ++x){
		out <<"    <collision>\n";
		for(int y = min_y; y <= max_y; ++y){
			for(uint i=1;i<=inst.num_vehicles();++i){
				out<< coll_line(grid(x,y),grid(x,y),i);
			}
		}
		out <<"    </collision>\n";
    }
    
	//edge  collisions: all with a "real" overlap for edges in bith directions
	for(int x = min_x; x <= max_x-1; ++x){
		out <<"    <collision>\n";
		for(int y = min_y; y <= max_y; ++y){
			for(uint i=1;i<=inst.num_vehicles();++i){
				//both direction
				out<< coll_line(grid(x,y),grid(x+1,y),i);
				out<< coll_line(grid(x+1,y),grid(x,y),i);
				//up if not y_min
				if(y>min_y){
					out<< coll_line(grid(x,y-1),grid(x+1,y-1),i);
					out<< coll_line(grid(x+1,y-1),grid(x,y-1),i);
				}
				if(y<max_y){
					out<< coll_line(grid(x,y+1),grid(x+1,y+1),i);
					out<< coll_line(grid(x+1,y+1),grid(x,y+1),i);
				}
				
			}
		
		}
		out <<"    </collision>\n";
	}
		
	out << "  </collisions>\n\n";	
	
			
	return true;
}


/*
Checks wether an instance is well suited for the Laser Sharing Problem.
Which means, job length is 0 or 1 and no coordinate is too big, say
less than 50 grid nodes.

Warn if jobs meet at start/end position?
*/
bool LaserSharingProblemWriter::check_instance(const Instance &inst, bool verbose) const{
	
	bool easy_instance = true;
	for(auto &job: inst){
		if( not (job.length()==0 or job.length()==1)){
			if(verbose) 
				cout<<"Waring: Job length of "<<job.length()<<" detected."<<endl;
			easy_instance = false;
		}
	}

	auto bbox = inst.get_bounding_box();
	int size = (bbox[2]-bbox[0])* (bbox[3]-bbox[1]);
	if( size > SMALL_GRID_SIZE){
		easy_instance = false;
		cout<< "Waring: grid is larger than advised.  Size:  "<<size<< endl;
	}
	
	 
	return easy_instance;
}
		
		
