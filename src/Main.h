#pragma once

#include <boost/algorithm/string.hpp>
#include <string>
#include <vector>
#include <cctype>
#include <algorithm>
#include <map>
#include <sstream>
#include <iostream>
#include <fstream>


const std::string BUILD_VERSION_{
    #include "version.txt"
    };

//all needed functions, their implementation can be found in Main.cpp
void read_instance(std::vector<std::string> argv);
void write_instance(std::vector<std::string> argv);
void test_mtsp_mip(std::vector<std::string> argv);
void insertion_heuristic(std::vector<std::string> argv);
void laser(std::vector<std::string> argv);
void single_tsp(std::vector<std::string> argv);
void test(std::vector<std::string> argv);
void batch(std::vector<std::string> argv);

template <class T,class U>
void printMapIndex( std::map<T,U> map){
	typename std::map<T, U>::const_iterator iter;
	for(iter = map.begin(); iter != map.end(); ++iter ) 
		std::cout << "\t- "<<iter->first << std::endl;
}

int process_args(std::vector<std::string> argv){
    std::cout<<std::boolalpha;

	//build function dictionary
	std::map<std::string,void (*)(std::vector<std::string>)> functionDict;
	//ADD NEW FUNCTIONS HERE (they have to parse ther arguments or call a usage)
	functionDict["read"] 		= &read_instance;
	functionDict["write"] 		= &write_instance;
	functionDict["mtsp_mip"] 	= &test_mtsp_mip;
	functionDict["insertion"]	= &insertion_heuristic;
	functionDict["test"] 	= &test;
	functionDict["laser_format"] 	= &laser;
	functionDict["single_tsp"]      = &single_tsp;
	functionDict["batch"]       = &batch;
	//at least one parameter must be given
	if(argv.size() < 1){
		std::cout<<"No function given, try one of these:"<<std::endl;
		printMapIndex(functionDict);
		std::cout<< "Version: "<< BUILD_VERSION_ << std::endl;
		return -1;
	}

    auto method = argv[0];
    std::vector<std::string> arguments;
    arguments.insert(begin(arguments),begin(argv)+1, end(argv));

	//functioncall or print all commands
	if( functionDict.find(method)==functionDict.end()){
		std::cout<<"No matching function found, try one of these:"<<std::endl;
		printMapIndex(functionDict);
		return -2;
	}else{
		// function call
		try{
        	functionDict[method](arguments);
    	}catch(std::invalid_argument &){
			std::cout<< "Given arguments are invalid for the choosen function. Exit." <<std::endl;
			return -3;
		}catch(std::out_of_range &oor){
			std::cout<<"The argument you have given it out of range for its type."<<std::endl;
		    std::cout << oor.what() << std::endl;
		    return -4;
		}
	}
	return 0;
}

int main(int argc, char** argv){
	//parse arguments
	std::vector<std::string> arguments;
	if(argc >= 2){
        std::string method =  argv[1];
	    boost::algorithm::to_lower(method);
        arguments.push_back(method);
	    for(int i = 2; i < argc; ++i)
		    arguments.push_back(argv[i]);
    }
    return process_args(arguments);
}

 


