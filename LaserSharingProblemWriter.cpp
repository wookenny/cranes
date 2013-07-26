#include "LaserSharingProblemWriter.h"

#include <iostream>
#include <iomanip>
#include <fstream>


using namespace std;



bool LaserSharingProblemWriter::write(const Instance &i, std::string filename, bool zipped) const{
	filename = add_suffix(filename, DEFAULT_SUFFIX);

	if( not zipped){
		std::ofstream file;
		file.open(filename.c_str());
		if( !file ) {
			std::cerr << "Error writing instance to "<<filename<<" ." << std::endl;
			return false;	
		}	

		//write instance
		write_file_unzipped(i,file);
	
		//end
		file << "\n";
		file.close();
	}
	else
	{ //zipped output file
		filename += ".gz";
		gzFile outfile = gzopen(filename.c_str(), "wb");
		if (!outfile) return false;

		//write
		write_file_zipped(i,outfile);
		//gzputs(outfile, "test\n");
		
		gzclose(outfile);
	}
	
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
 - cycletime: makespan angeben, fürden die lösung gesucht wird
 - jobs auf gridpunkte legen
 - 
*/
bool LaserSharingProblemWriter::write_file_unzipped
								(const Instance &inst, std::ofstream &out) const{
								
	out<< "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>"
	  <<"<lsp cycletime=\"330.000000000000\""
	  <<" xmlns=\"http://www.wm.uni-bayreuth.de/fileadmin/Cornelius/XMLSchema/lsp"
	  <<"\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\""
	  <<" xsi:schemaLocation=\"http://www.wm.uni-bayreuth.de/fileadmin"
	  <<"/Cornelius/XMLSchema/lsp http://www.wm.uni-bayreuth.de/fileadmin"
	  <<"/Cornelius/XMLSchema/lsp.xsd\">\n\n";							
	
	//robots
	out << "  <robots>\n";
	for(uint i=1;i<=inst.num_vehicles();++i){
		out<<"    <robot name=\""<< i <<"\">\n";
		out<<"      <home name=\"h"<< i <<"\">\n";
		out<<"    </robot>\n";
	}
    out<<"\n";
    			
    //jobs			
    out << "  <jobs>\n";
	for(const auto &job: inst){
		out << "    <job direction=\"fixed\" name=\"J"<<job.num()<<"\">\n";
		out << "      <aPos name=\"J"<<job.num()<<"A\"/>\n";
		out << "      <bPos name=\"J"<<job.num()<<"B\"/>\n";
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
    		add_dist("G"+to_string(x)+"_"+to_string(y),
    				 "G"+to_string(x)+"_"+to_string(y), 0);	
    	}
    }
    //0 distance edges for the connection between grid and jobs/depots
    
    out << "  </distances>\n\n";
        
    //write collisions

    			
	return true;
}


bool LaserSharingProblemWriter::write_file_zipped
									(const Instance &, gzFile &) const{
	//TODO: string version with above thing, do not copy the code!
	return true;
}
		
		
