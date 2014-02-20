#include "Main.h"

#include "Common.h"
#include <algorithm>
#include <unordered_map>
#include <memory>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


#if __clang__ 
    //no omp.h for clang++
#else
    #include <omp.h>
#endif
#include <ctime>
#include <chrono>

#include "generalizedVRP_MIP.h"
#include "m_TSP_MIP.h"
#include "independent_TSP_MIP.h"
#include "Instance.h"
#include "InsertionHeuristic.h"
#include "LaserSharingProblemWriter.h"
#include "SingleCraneTSP_Solver.h"

using namespace std;
using namespace boost::filesystem;

void read_instance(vector<string> argv){
	
	if (argv.size()!=1){
		cout<<"read [file] \n \tRead the input file and print the instance."<<endl;
		return;
	}
	
	//read file and give statistics
	Instance inst(argv[0]);
	cout<< inst <<endl;
}
//TODO: delete old stub
/*
void print_random_instance(vector<string> argv){
	if (argv.size()<2 || argv.size()>4){
		cout<<"random [k] [n] <s> <filename> \n \tGenerates a random instance with k vehicles, \n\tn jobs and with seed s. Default seed is 0.\n\twrites the instance to te given filename"<<endl;
		return;
	}
	
	unsigned int seed = 0;
	Instance i;
	i.set_num_vehicles(stoi(argv[0]));
	
	if(argv.size()>2)
		seed = stoi(argv[2]);
	i.generate_random_depots(-100,100,-10,10,seed);
	i.generate_random_jobs(stoi(argv[1]),-100,100,-10,10,seed);
	
	cout<< i <<endl;

	if(argv.size()>3){
		string filename(argv[3]);
		if(filename.length()<5 or filename.substr(filename.length()-5,5) != ".2dvs")
			filename += ".2dvs";
		i.writeToFile(filename);
		cout<<"Written instance to file "<<filename <<endl;
	}
}
*/
//==============================================================================

void write_instance(vector<string> argv){
     try {
        //define all parameters to set
        uint num_vehicles;
        unsigned int seed;
        string filename = "";
        uint num_jobs=0;
        uint safety_dist; 
        int verbosity;
        bool show_instance;
        int min_x, max_x, min_y, max_y;
        double percentage_driveby;
        double min_length_x, min_length_y, max_length_x, max_length_y;
        
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("verbosity,v", po::value<int>(&verbosity)->default_value(1), 
              "verbosity level")
            ("seed,s", po::value<unsigned int>(&seed)->default_value(0), 
              "set seed for random samples")
            ("filename,f", po::value<string>(&filename), 
             "2DVS file for heuristic")
            ("k,k", po::value<uint>(&num_vehicles)->default_value(2)->required(), 
             "set number of vehicles/cranes")
            ("n,n", po::value<uint>(&num_jobs)->required(), 
             "set number of jobs to generate in the instance")
            ("print,p", po::value<bool>(&show_instance)->default_value(0), 
            "print generated instance to console")
            ("minx,x", po::value<int>(&min_x)->default_value(0),
                                             "minimal position on the x-axis")
            ("maxx,X", po::value<int>(&max_x)->default_value(400), 
                                             "maximal position on the x-axis")
            ("miny,y", po::value<int>(&min_y)->default_value(0),
                                             "minimal position on the y-axis")
            ("maxy,Y", po::value<int>(&max_y)->default_value(40), 
                                             "maximal position on the y-axis")
            ("safetydist", po::value<uint>(&safety_dist)->default_value(0), 
             "set safetydistance between vehicles")
            ("driveby,d", po::value<double>(&percentage_driveby)
                                                    ->default_value(0), 
             "set percentage of drive-by jobs in relation with all jobs."
              "Valid range: [0,100].")
             ("minlengthx", po::value<double>(&min_length_x)
                                                    ->default_value(0), 
             "set minimal length along x-axis of jobs in percentage of storage size. "
             "Valid range: [0,100].") 
             ("maxlengthx", po::value<double>(&max_length_x)
                                                    ->default_value(100), 
             "set maximal length along x-axisof jobs in percentage of storage size. "
             "Valid range: [0,100].")
             ("minlengthy", po::value<double>(&min_length_y)
                                                    ->default_value(0), 
             "set minimal length along y-axis of jobs in percentage of storage size. "
             "Valid range: [0,100].") 
             ("maxlengthy", po::value<double>(&max_length_y)
                                                    ->default_value(100), 
             "set maximal length along y-axis of jobs in percentage of storage size. "
             "Valid range: [0,100].")  
             ;

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style(   po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")){
            cout<<"Generates a 2DVS instances and writes it to a file. ";
            cout<<"If no filename is give, the generated instance will ";
            cout<<"be shown in the console." <<"\n";
            cout<<boolalpha << desc;
            cout<<"required: 'n' and 'k'\n";
            return;
        }
        

        po::notify(vm); 
        //verify settings
        if( min_x > max_x or min_y > max_y){
            cerr<< "Invalid bounds given: '"<<min_x<<" <= x <= "<<max_x;
            cerr<< "' and '"<< min_y<< " <= y <= "<<max_y<<"'\n";
            return;
        }   
        if( min_length_x<0 or min_length_x>100 or 
            max_length_x<0 or max_length_x>100 or
            min_length_y<0 or min_length_y>100 or 
            max_length_y<0 or max_length_y>100 or
            min_length_x > max_length_x or min_length_y > max_length_y)
        {
            cerr<< "Invalid job length bounds given: \n'";
            cerr<< min_length_x<<" <= x length <= "<<max_length_x<<"'\n";
            cerr<< min_length_y<<" <= y length <= "<<max_length_y<<"'\n";
            return;
        }
        if(percentage_driveby<0 or percentage_driveby>100){
            cerr<< "Percentage of drive-by jobs not in valid range: ";
            cerr<< percentage_driveby<<"\n" <<endl;
            return;
        }
        
        Instance i(num_vehicles);
        i.set_safety_distance(safety_dist);

        i.generate_random_depots(min_x,max_x,min_y,max_y,seed);
        int num_driveby = static_cast<int>(0.01*percentage_driveby*num_jobs);
        i.generate_random_driveby_jobs(num_driveby,min_x,max_x,
                                                   min_y,max_y,seed);
        i.generate_random_bounded_jobs(num_jobs-num_driveby,
                                       min_x,max_x, min_y,max_y,
                                       min_length_x, max_length_x,
                                       min_length_y, max_length_y,
                                       seed);                                           
	    
	    
        if(show_instance or not vm.count("filename") or verbosity>=2)
            cout<< "Generated instance:\n"<< i <<endl;

           
        //print statistics     
        stringstream params;
        params << "\nParameter: \nvehicles: "<<num_vehicles<<"\n";
        params << "jobs: "<<num_jobs<<"\n";    
        params << "percentage of drive-by: "<<percentage_driveby<<"% -> ";
        params << num_driveby<<" drive-by jobs\n"; 
        params << "positions: ["<<min_x<<":"<<max_x<<"] x [";
        params << min_y<<":"<<max_y<<"]"<<"\n";  
        params << "length bound: ["<<min_length_x<<"% - "<<max_length_x<<"%] x [";
        params << min_length_y<<"% - "<<max_length_y<<"%]"<<"\n";  
        params << "safety distance: "<<safety_dist<<"\n";
        params << "seed: "<<seed<<"\n";
        
        if(verbosity>=2){   
            cout<<params.str();
        }
        
        //write file
        if(vm.count("filename")){
            if(filename.length()<5 or filename.substr(filename.length()-5,5) != ".2dvs")
		        filename += ".2dvs";
		    string comment = params.str();
		    path p{filename.c_str()};
		    if( exists(filename) ){
		        cerr<<"Error: file '"<<filename<<"' already exists!\n";
		        return;
		    }
		    i.writeToFile(filename,comment);
		        
            if(verbosity>=1)
                cout<<"Written instance to file "<<filename <<endl; 
        }    

	}catch(boost::program_options::required_option& e){
	    cerr << " " << e.what() << "\n";
	    cerr << " Try --help for all parameters.\n";
    }catch(exception& e) {
        cerr << " " << e.what() << "\n";
        return;
    }catch(...) {
        cerr << "Exception of unknown type!\n";
        return;
    }
    return;
}

//==============================================================================

void test_mtsp_mip(vector<string> argv){
	if (argv.size()< 1 or argv.size()>5 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"test_mip <2dvs file> <collision constr., default = true>" 
			 <<" <LP relaxation., default = false> <TSP-type, 0 = condensed k-TSP, 1 = independent k-TSP, default = 1>\n Runs some tests on the mip formulation!"<<endl;
		return;
	}
	
	//set default parameter and parse given value
	bool collisions = true;
	bool lp_relax = false;
	int mip_type = 1;
	
	unordered_map<string,bool> string_to_bool =  {{"t",true},{"true",true},
			{"1",true},{"yes",true},{"f",false},{"false",false},
			{"0",false},{"no",false},{"y",true},{"n",false} };

	if(argv.size()>1)
		if(string_to_bool.find(argv[1])!=string_to_bool.end())
			collisions = string_to_bool[argv[1]];	
	if(argv.size()>2)
		if(string_to_bool.find(argv[2])!=string_to_bool.end())
			lp_relax = string_to_bool[argv[2]];	
	if(argv.size()>3)
		mip_type = stoi(argv[3]);


	Instance i{argv[0]};
			
	unique_ptr<generalizedVRP_MIP> mip_ptr;
	if(1==mip_type) //remember: explicit std::move here because of rvalue
		mip_ptr = unique_ptr<generalizedVRP_MIP>(new independent_TSP_MIP(i));
	else
		mip_ptr = unique_ptr<generalizedVRP_MIP>(new m_TSP_MIP(i));
	
	mip_ptr->set_debug(false);
	mip_ptr->set_collision(collisions);
	mip_ptr->set_LP(lp_relax);
		
	
	//run heuristic, use it as starting solution	
    cout<<"heuristic solution:"<<endl;
	InsertionHeuristic heur(true);
	heur.set_runs(20);
	auto sol = heur(i); 
	cout<<"Heristic Solution valid: "<<i.verify(sol)<<endl;
	cout<<"Makespan: "<<i.makespan(sol)<<endl;
	cout<<sol<<endl;		
	
	//mip_ptr->set_start_solution(sol);	
	mip_ptr->set_fixed_makespan( .5*i.makespan(sol));
	Tours &&t = mip_ptr->solve();

	cout<<boolalpha;
	cout<<"MIP-Solution valid: "<<i.verify(t)<<endl;
	cout<<"Makespan: "<<i.makespan(t)<<endl;
	cout<<t<<endl;

	

	if(not i.verify(sol) or i.makespan(sol) < i.makespan(t))
		cout<<"WARNING!: SOMETHING IS TERRIBLY BAD. MIP WORSE THAN HEURISTIC!"<<endl;
}


void insertion_heuristic(std::vector<std::string> argv){
     try {
        //define all parameters to set
        int verbosity;
        unsigned int seed;
        string filename = "";
        bool debug;
        int k=0;
        int n=0;
        bool local_search;
        int t_max;
        int n_threads;
        bool use_assign;
        bool stop_at_better;
        int n_runs;
        uint safety_dist; 
        
        po::options_description desc("Allowed options");

        desc.add_options()
            ("help,h", "produce help message")
            ("verbosity,v", po::value<int>(&verbosity)->default_value(1), 
              "verbosity level")
            ("seed,s", po::value<unsigned int>(&seed)->default_value(0), 
              "set seed for random samples")
            ("filename,f", po::value<string>(&filename), 
             "2DVS file for heuristic")
            ("debug,d","set debug mode for instances")
            ("k", po::value<int>(&k), 
             "set number of vehicles/cranes")
            ("n", po::value<int>(&n), 
             "set number of jobs to generate in the instance")
            ("localsearch,l", 
             "enable or disable the local search")
            ("timelimit,t", po::value<int>(&t_max)->default_value(-1), 
             "set a timelimit for the heuristic, -1 means not limit at all") 
            ("threads", po::value<int>(&n_threads)->default_value(-1), 
             "set the number of parallel threads used, -1 means an automatic value is found")  
            ("safetydist", po::value<uint>(&safety_dist), 
             "set safetydistance between vehicles, overrides the setting via 2dvs file")
            ("assign,a","use an assignment or not, in the heuristic")
            ("stop-at-better,b", "stopping scan of neigbourhood in a single local search step, if a better neighbor was found")
            ("runs,r", po::value<int>(&n_runs)->default_value(1), "number of runs using on a single input")    
        ;

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style(   po::command_line_style::unix_style)
                    .run(), vm);
        po::notify(vm);    

        //react on som settings
        if (vm.count("help") ){
            cout<<boolalpha << desc << "\n";
            return;
        }
        use_assign      = (vm.count("assign")>0);
        stop_at_better  = (vm.count("stop-at-better")>0);
        debug           = (vm.count("debug")>0);
        local_search    = (vm.count("localsearch")>0);


        //after parsing, execute the selected method    
	    Instance i;   
	    if(vm.count("filename")){
	        i = Instance{filename};
	        if( vm.count("k") or vm.count("n")) 
	            cout<<"Warning: Ignoring given number of vehicles and jobs!"<<endl;
	    }else{
	        if( not vm.count("k") and not vm.count("n")) {
	            cout<< "Number of jobs and vehicles not given."
	                  << " No filename given. Usage:\n"<<desc<<endl;
	            return;
	        }
	            
	        i.set_num_vehicles(k);
	        i.generate_random_depots(0,100,0,20,seed);
	        i.generate_random_jobs(n,0,100,0,20,seed); 
	    }
	    
	    i.debug(debug);
	    if (vm.count("safetydist")>0)
	        i.set_safety_distance(safety_dist);
        
        InsertionHeuristic heur(local_search);
        heur.set_seed(seed);
        heur.set_verbosity(verbosity);
        heur.set_timelimit(t_max);
        heur.set_num_threads(n_threads);
        heur.set_use_assignment(use_assign);
        heur.set_stop_at_better(stop_at_better);
        heur.set_runs(n_runs);
        
        //solve it
        auto sol = heur(i);
	    cout<<endl;
	    if(i.num_jobs()<=20)
	        cout<<"\n"<< sol <<endl;
	    cout<<"makespan of solution: "<<i.makespan(sol)<<endl;

        if(Instance::num_checks>0)
            cout<< "Solutions checked in total: "<<Instance::num_checks <<endl;

    }catch(exception& e) {
        cerr << " " << e.what() << "\n";
        return;
    }catch(...) {
        cerr << "Exception of unknown type!\n";
        return;
    }
    return;
}


/*
void test_mip(vector<string> argv){
	if (argv.size()>5 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"test_mip <n> <k> <coll.> <LP> <seed>\n Runs some tests on the mip formulation!";
		cout<<"\n\tn: number of jobs(default = 4)\n\tk: number of vehicles(default = 2)"<<endl;
		cout<<"\tcoll.: collision-avoidance constraints adding (default = true)"<<endl;
		cout<<"\tLP: just solve the LP relaxation(default = false)"<<endl;
		cout<<"\tseed: seed used to genenerate the jobs.(default = time(0))"<<endl;
		return;
	}
	//set default parameter and parse given values
	int k = 2;
	int jobs = 4;
	bool collision = true,LP = false;
	int seed = time(0);
	
	//parse the given ones
	if(argv.size()>0)
		jobs = stoi(argv[0]);
	if(argv.size()>1)
		k = stoi(argv[1]);
	unordered_map<string,bool> string_to_bool =  {{"t",true},{"true",true},
			{"1",true},{"yes",true},{"f",false},{"false",false},
			{"0",false},{"no",false},{"y",true},{"n",false} };

	if(argv.size()>2)
		if(string_to_bool.find(argv[2])!=string_to_bool.end())
			collision = string_to_bool[argv[2]];
	if(argv.size()>3)
		if(string_to_bool.find(argv[3])!=string_to_bool.end())
			LP = string_to_bool[argv[3]];		
	if(argv.size()>4)
		seed = stoi(argv[4]);
	//give some informations
	cout<<boolalpha;
	cout<< "Solving a 2D-VS instance with the following settings:\n";
	cout<< "Jobs: "<<jobs<<"\nVehicles: "<<k;
	cout<<"\nAdding collision avoidance constr.: "<<collision;
	cout<<"\nSolving LP relaxation.: "<<LP<<endl;
		
	Instance i(k);
	//i.generate_random_depots(-10, 10, -10, 10, 0);
	for(int j=0; j<k;++j)
		i.add_depotposition(array<int, 2>{{0,0}});

	i.generate_random_jobs(  jobs, -10, 10, -10, 10, seed);
	Tours &&t = i.get_MIP_solution(collision, LP);

	
	cout<<i<<endl;
	cout<<boolalpha;
	if(not LP) cout<<"MIP-Solution valid: "<<i.verify(t)<<endl;
	cout<<"Makespan: "<<i.makespan(t)<<endl;
	cout<<t<<endl;
}
*/


//TODO: REMOVE THIS AFTER IMPLEMENTING A CHRISTOFIDES-LIKE Heuristic 
void test(std::vector<std::string> argv){
	if (argv.size()<2 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"Testfunction with a two to three arguments."<<endl;
		return;
	}
	
	int number_of_jobs = stoi(argv[0]);
	int runs = stoi(argv[1]);
	int completed = 0;
	int seed = 0;//time(0);
	
	if(argv.size()>2)
		seed = stoi(argv[2]);

	//stop startingtime
	using std::chrono::duration_cast;
	using std::chrono::microseconds;
	using std::chrono::system_clock;

	system_clock::time_point start = system_clock::now();
	
	#pragma omp parallel for
	for(int r=0; r<runs;++r){
		++completed;
		cout << "\r";
		Instance i(2);
		i.generate_random_jobs(  number_of_jobs, -10, 10, -10, 10, seed+r);
		i.add_depotposition(array<int, 2>{{-5,0}});
		//i.add_depotposition(array<int, 2>{{ -3,0}});
		//i.add_depotposition(array<int, 2>{{  3,0}});
		i.add_depotposition(array<int, 2>{{ 5,0}});

		#if __clang__ 
		if(true){
		#else
		if(0==omp_get_thread_num()){ //CLANG does not like this line	
		#endif
			auto time = duration_cast<std::chrono::seconds>(system_clock::now()- start).count();
			double speed =  completed/(1.0*time);
			auto remaining = (runs-completed)/speed;
			cout<<" Runs remaining: "<<runs-completed;
			if(1.*completed/runs > .1)
				cout<<"  time left: "<<(int)remaining+1<<" seconds                      ";

			//cout<<"    Testing instance with seed: "<<seed+r<<"     ";
			cout.flush();
		}		
				
		SingleCraneTSP_Solver tsp;
		auto tsp_result = tsp(i);
		Tours t = std::get<1>(tsp_result);
		if(not i.verify(t) ){
			cout<<"Found instance with invalid solution:\n"<<i<<endl;
			cout<<"Seed: "<<seed+r<<endl;
			cout<<"tours: "<<t<<endl;
			//break;
		}
		
	}
	
	cout<< "\n\nAll tested solutions ok."<<endl;
}

void laser(std::vector<std::string> argv){
	if (argv.size()<2 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"laser_format [2dvs] [lsp] <zipped>\n\tConverts a 2dvs file [2dvs] into a LaserSharingProblem file[lsp]. If a third argument is given, it will compess the outputfile as gz."<<endl;
		return;
	}
	
	string twoDVS = argv[0];
	string lsp    = argv[1];

	Instance i(twoDVS);
	LaserSharingProblemWriter lsp_writer;
	if(lsp_writer.write(i, lsp,argv.size()>2)){
		cout<< "Written "<<twoDVS<<" instance as a laser sharing problem to file "
			<< lsp <<endl;
	}else{
		cout<< "ERROR: Couldn't write "<<twoDVS<<" instance as a laser sharing problem to file "
			<< lsp <<endl;
	}
}

void single_tsp(std::vector<std::string> argv){
     try {
        //define all parameters to set
        int verbosity;
        string filename = "";

        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("filename,f", po::value<string>(&filename)->required(), 
             "2DVS file")
            ("localsearch,l", 
             "enable or disable the local search")
            ("verbosity,v",po::value<int>(&verbosity)->default_value(0));

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style( po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")){
            cout<<"Calculates a lower bound for the given file.";
            cout<<"The bound is given by a TSP formulation, which is solved optimally via Concorde.\n"; 
            cout<<boolalpha << desc << "\n";
            cout<<"required: 'f'"<<endl;
            return;
        }
        po::notify(vm); 
        
        //after parsing, execute the selected method    
        SingleCraneTSP_Solver solver;
        Instance i(filename);
        solver.set_verbosity(verbosity);
        if(vm.count("localsearch"))
            solver.set_local_search(true);
	    auto tsp_result = solver(i);
	
	    Tours t = std::get<1>(tsp_result);
	    double bound = std::get<0>(tsp_result);

        cout<<"makespan of a tour based on optimal single vehicle tour: "<<i.makespan(t)<<endl;
        cout<<"TSP bound: "<<bound<<endl;
        cout<<"ratio: "<<i.makespan(t)*100/bound<<"%"<<endl;
        if (i.makespan(t)/bound > i.num_vehicles()){
             cerr<<"WARNING: ratio is above number of vehicles(";
             cerr<<i.num_vehicles()<<")"<<endl; 
        }  

    }catch(boost::program_options::required_option& e){
	    cerr << " " << e.what() << "\n";
	    cerr << " Try --help for all parameters.\n";
    }catch(exception& e) {
        cerr << " " << e.what() << "\n";
        return;                    
    }catch(...) {
        cerr << "Exception of unknown type!\n";
        return;
    }
    return;
  
       
}


/**
 Recursive Method to insert all arguments and run the matching method!
**/
void recursive_replace_call(string command, uint pos, 
                                    const vector<vector<string>>& replacement){
    //all parameter replaced => call program
    if(pos>=replacement.size()){
        cout<<"--------------------------------------\n";
        cout<<"calling: "<<command;
        cout<<"\n--------------------------------------"<<endl;
        int return_code = process_args(split(command));   
        if(return_code != 0){
            cout<< "\nLast call was not valid! Ending batch runs.\n" <<endl;
            exit(0);
        } 
    }
    else{//choose new replacement
        for(auto s: replacement[pos]){
            string new_command =  command;
            replaceAll(new_command, "#"+to_string(pos+1), s);
            recursive_replace_call(new_command,pos+1,replacement);        
        }

    }
}


void batch(vector<string> argv){
    if (argv.size()<2 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help")) ){
		cout<<"batch \"<command> #1 <param> #2 <param> #1...\" <replace1> <replace2>... \n\tRuns the command in batch mode."<<endl;
		return;
	}
	
	string command = argv[0];
	vector<vector<string>> replace_lists;
	
    //build intervals for numbers or lists of files, using wildcard '*'			
 	for(uint i=1; i< argv.size(); ++i){
        auto interval = create_interval(argv[i]);
        if(!interval.empty())
            replace_lists.push_back(interval);
	    else{ //try to find wildcard filelist
	        auto files =  find_files(argv[i]);
	        replace_lists.push_back(files);  
	    }
	}
    
    /*
    for(auto l: replace_lists){
        for(auto e: l){
            cout<<e <<" ";
        }
        cout<< l.size()<<endl;
    }
   //creating all calls!
  */
  
  int pos = 0;
  recursive_replace_call(command,pos,replace_lists);   
} 


