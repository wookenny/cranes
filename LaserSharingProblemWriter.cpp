#include "LaserSharingProblemWriter.h"

#include <iostream>
#include <fstream>


using namespace std;



bool LaserSaringProblemWriter::write(const Instance &i, std::string filename, bool zipped) const{
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

std::string LaserSaringProblemWriter::add_suffix(string s,string suffix) const{
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
 - was ist die cycletime?
 - fixed direction = von a nach b?
*/
bool LaserSaringProblemWriter::write_file_unzipped
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
		out << "    <job direction=\"free\" name=\"J"<<job.num()<<"\">\n";
		out << "      <aPos name=\"J"<<job.num()<<"A\"/>\n";
		out << "      <bPos name=\"J"<<job.num()<<"A\"/>\n";
		out << "      <robots>\n";
		for(uint i=1;i<=inst.num_vehicles();++i)
			out << "        <robot name=\"" << i << "\"/>\n";
		out << "       </robots>\n";
    	out << "    <job>\n"; 		
    }
    out << "  </jobs>\n\n";

    
    //write distances 
        
    //write collisions

    			
	return true;
}


bool LaserSaringProblemWriter::write_file_zipped
									(const Instance &inst, gzFile &outfile) const{
	//TODO: string version with above thing, do not copy the code!
	return true;
}
		
		
