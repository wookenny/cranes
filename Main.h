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


//all needed functions, their implementation can be found in Main.cpp
void read_instance(std::vector<std::string> argv);
void print_random_instance(std::vector<std::string> argv);
void test_mtsp_mip(std::vector<std::string> argv);
void insertion_heuristic(std::vector<std::string> argv);
void laser(std::vector<std::string> argv);

void test(std::vector<std::string> argv);

template <class T,class U>
void printMapIndex( std::map<T,U> map){
	typename std::map<T, U>::const_iterator iter;
	for(iter = map.begin(); iter != map.end(); ++iter ) 
		std::cout << "\t- "<<iter->first << std::endl;
}


int main(int argc, char** argv){
    std::cout<<std::boolalpha;

	//build function dictionary
	std::map<std::string,void (*)(std::vector<std::string>)> functionDict;
	//ADD NEW FUNCTIONS HERE (they have to parse ther arguments or call a usage)
	functionDict["read"] 		= &read_instance;
	functionDict["random"] 		= &print_random_instance;
	functionDict["mtsp_mip"] 	= &test_mtsp_mip;
	functionDict["insertion"]	= &insertion_heuristic;
	functionDict["test"] 	= &test;
	functionDict["laser_format"] 	= &laser;
	//at least one parameter must be given
	if(argc < 2){
		std::cout<<"No function given, try one of these:"<<std::endl;
		printMapIndex(functionDict);
		return 0;
	}

	//parse arguments
	std::string method =  argv[1];
	boost::algorithm::to_lower(method);
	std::vector<std::string> arguments;
	for(int i = 2; i < argc; ++i)
		arguments.push_back(argv[i]);

	//functioncall or print all commands
	if( functionDict.find(method)==functionDict.end()){
		std::cout<<"No matching function found, try one of these:"<<std::endl;
		printMapIndex(functionDict);
	}else{
		// function call
		try{
        	functionDict[method](arguments);
    	}catch(std::invalid_argument &){
			std::cout<< "Given arguments are invalid for the choosen function. Exit." <<std::endl;
		}catch(std::out_of_range &oor){
			std::cout<<"The argument you have given it out of range for its type."<<std::endl;
		std::cout << oor.what() << std::endl;
		}

	}
}

 


