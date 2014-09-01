#include "Main.h"
#include "Common.h"
#include <algorithm>
#include <unordered_map>
#include <memory>
#include <iomanip>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/any.hpp>

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
#include "MipInsertionHeuristic.h"
#include "LaserSharingProblemWriter.h"
#include "SingleCraneTSP_Solver.h"
#include "SeparationHeuristic.h"
#include "ConsolidationHeuristic.h"

using namespace std;
using namespace boost::filesystem;
using std::chrono::minutes;

void read_instance(vector<string> argv){
	
	if (argv.size()!=1){
		cout<<"read [file] \n \tRead the input file"
			   " and print the instance."<<endl;
		return;
	}
	
	//read file and give statistics
	Instance inst(argv[0]);
	cout<< inst <<endl;
}

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
             "2DVS filemname for the new file")
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


void run_mip(vector<string> argv){
    try {
        //define all parameters to set
        int verbosity;
        string filename = "";
        int k,n; 
        uint mip_type = 1;
        uint runs;
        uint seed;
        uint fixed_makespan;
        double timelimit;

        po::options_description desc("Allowed options");

        desc.add_options()
            ("help,h", "produce help message")
            ("verbosity,v", po::value<int>(&verbosity)->default_value(0), 
              "verbosity level")
            ("filename,f", po::value<string>(&filename), 
             "2DVS file to solve via MIP")
            ("k", po::value<int>(&k), 
             "set number of vehicles/cranes")
            ("n", po::value<int>(&n), 
             "set number of jobs to generate in the instance")
            ("seed,s", po::value<unsigned int>(&seed)->default_value(0), 
              "set seed for random samples")
            ("debug,d","set debug mode for instances")
            ("timelimit,T", po::value<double>(&timelimit)->default_value(0),
                            "time limit in minutes for the MIP solver")
            ("tighteningconstraints,g", "add tightening constraints for the"
                                       " sum of arcs")
            ("miptype,t",po::value<uint>(&mip_type)->default_value(1), 
              "type of MIP: 0 = two-indexed VRP, 1 = three-indexed VRP")
            ("no_collsion_avoidance,c", "disables the collision avoidance constraints")
            ("lp_relaxation,l", "solve the LP relaxation of the IP formulation")
            ("makespan,m", po::value<uint>(&fixed_makespan)->default_value(0), 
              "set a fixed makespan to find a solution not exceeding it")
            ("subtourcuts,S","enable subtour cuts as user cuts.")
            ("assignment,a","uses a random assigment for all jobs.")
            ("intial,i", "using a heuritic to find an initial startting "
                                                        "solution for he mip")
            ("runs,r", po::value<uint>(&runs)->default_value(20), 
              "number of runs to find an initial solution using the "
              "insertion heuristic")
        ;
        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style(   po::command_line_style::unix_style)
                    .run(), vm);
 

        //react on som settings
        if (vm.count("help") ){
            cout<< "Solves a 2DVS instances via MIP. Either with the two- or"
                    " the three-indexed VRP MIP.\n"
                    "Collision constraints, LP relaxation and user cuts can be" 
                    "switched on or off." <<endl;
            cout<<boolalpha << desc << "\n";
            return;
        }
        
        po::notify(vm);   
        
         //after parsing, execute the selected method    
        Instance i; 
        i.debug( vm.count("debug")>0 ); 
        if(vm.count("filename")){
            i = Instance{filename};
            if( vm.count("n")) 
                cout<<"Warning: Ignoring given number of jobs!"<<endl;
            if( vm.count("k")){
                cout<<"Warning: Ignoring number of vehicles in file!"<<endl;
                i.set_num_vehicles(k);
            }
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
        

        //setup MIP and run several tests.
        unique_ptr<generalizedVRP_MIP> mip_ptr;
        if(1==mip_type){ //remember: explicit std::move here because of rvalue
            cout<<"Using the three-indexed VRP MIP"<<endl;
            mip_ptr = unique_ptr<generalizedVRP_MIP>(new independent_TSP_MIP(i));
        }else{
            cout<<"Using the two-indexed VRP MIP"<<endl;
            mip_ptr = unique_ptr<generalizedVRP_MIP>(new m_TSP_MIP(i));
        }
        mip_ptr->set_LP( vm.count("lp_relaxation")>0 );    
        mip_ptr->set_collision(vm.count("no_collsion_avoidance")==0);        
        mip_ptr->set_debug( vm.count("debug")>0 );
        bool valid_solution = vm.count("lp_relaxation")==0;
        valid_solution = valid_solution and (vm.count("collisions")==0);
        mip_ptr->use_subtour_cuts(vm.count("subtourcuts") > 0 );
        cout<< "subtour cuts: "<<(vm.count("subtourcuts") > 0 )<<endl;
        mip_ptr->set_tightening_cons(vm.count("tighteningconstraints")>0 );
        cout<< "tightening cons: "<<(vm.count("tighteningconstraints")>0)<<endl;

        mip_ptr->set_timelimit(timelimit);
        mip_ptr->set_silent(verbosity<1);
        if(timelimit>0)
            cout<< "timelimit: "<< minutes_to_string(timelimit)<<endl;
        if(fixed_makespan != 0)
            mip_ptr->set_fixed_makespan(fixed_makespan);        
            
        //run heuristic, use it as starting solution    
        Tours sol{i.num_vehicles()};
        if(runs > 0 and vm.count("assignment")==0 and vm.count("intial")>0){
            InsertionHeuristic heur(true);
            heur.set_runs(runs);
            sol = heur(i); 
            assert(i.verify(sol));
            cout<<"Makespan of heuristic solution: "<<i.makespan(sol)<<endl;
            //cout<<sol<<endl;        
            mip_ptr->set_start_solution(sol);   
        }

        
        
        
        //if assignment set
        if (vm.count("assignment") > 0) { 
            auto a = random_assignment(i.num_jobs(),
                                            0, i.num_vehicles()-1, seed);
            mip_ptr->set_assignment(a);
        }
        
        auto mip_sol = mip_ptr->solve();
        Tours t = get<0>( mip_sol );
        double objective = get<1>( mip_sol );
        if(i.num_jobs()<15 and verbosity > 0)
            cout<< "found tour: \n"<<t<<endl;

        cout<<boolalpha;

        if(valid_solution){
                cout<<"MIP-Solution valid: "<<i.verify(t)<<"\n";
                cout<<"Makespan: "<<i.makespan(t)<<"\n";
                cout<<"MIP Objective: "<<objective<<endl;
                cout<<"Running time: "
                    <<duration_to_string(mip_ptr->get_runningtime()) <<endl;
        }else{
            cout<<"Running time: "
                    <<duration_to_string(mip_ptr->get_runningtime()) <<endl;
            cout<< "Objective: "<<objective<<endl;           
        }  
        
        if( not sol.empty() and fixed_makespan==0 and valid_solution){
            if( not i.verify(sol) or  i.makespan(sol) < i.makespan(t) )
            cout<<"WARNING!: SOMETHING IS TERRIBLY BAD. "
                  "MIP WORSE THAN HEURISTIC!"<<endl;
         }         
    }catch(exception& e) {
        cerr << " " << e.what() << "\n";
        return;
    }catch(...) {
        cerr << "Exception of unknown type!\n";
        return;
    }
    return;
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
            ("verbosity,v", po::value<int>(&verbosity)->default_value(0), 
              "verbosity level")
            ("seed,s", po::value<unsigned int>(&seed)->default_value(0), 
              "set seed for random samples")
            ("filename,f", po::value<string>(&filename), 
             "2DVS file for heuristic")
            ("debug,d","set debug mode for instances")
            ("k,k", po::value<int>(&k), 
             "set number of vehicles/cranes")
            ("n,n", po::value<int>(&n), 
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
	    if(i.num_jobs()<=20 and verbosity > 0)
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




//TODO: REMOVE THIS AFTER IMPLEMENTING A CHRISTOFIDES-LIKE Heuristic 
void test(std::vector<std::string> argv){
	if (argv.size() >0 and (argv[0]=="h" or argv[0]=="help") ){
		cout<<"Testfunction with some arguments."<<endl;
		return;
	}

    uint seed = 0;
    bool once = true;
    while(true){
        uint num_jobs = 5;
        uint num_vehicles = 3;
        Instance i(num_vehicles);
        i.generate_random_depots(0,100,0,0,seed);
        i.generate_random_jobs(num_jobs,0,100,0,0,seed);

          
        //run mip
        auto  mip = independent_TSP_MIP(i);
        mip.set_debug( false );
        mip.set_collision( true );
        mip.set_LP( false );
        mip.set_silent(true);
        auto opt_tour = get<0>(mip.solve());
        double opt = i.makespan(opt_tour);
        if(!i.verify(opt_tour)){

            std::cout<<"\nW: invalid opt. sol!"<<std::endl;
            std::cout<<"seed: "<<seed<<"\n";
            std::cout<<i<<std::endl;
            std::cout<<"Sol:"<<opt_tour<<std::endl;
            return;
        }

        //run insertion on overy permutation
        std::vector<uint> perm;
        for(uint j=0; j<num_jobs;++j)
            perm.push_back(j);
        std::string max_assig= "";
        for(uint j = 0; j<num_jobs;++j)
            max_assig += std::to_string(num_vehicles-1);

        double min_insert = -1;
        do{

            //for every assignment: 
            for(int j=0; j<= std::stoi(max_assig,0,num_vehicles);++j){
                
                std::vector<uint> assign;
                std::string assi = itos(j,num_vehicles);

                for(uint l=0; l<assi.size();++l){
                    uint append = std::stoi(assi.substr(l,1),0,num_vehicles);
                    assign.push_back(append);
                }

                for(uint l = assign.size(); l<num_jobs;++l)
                    assign.insert(assign.begin(), 0);
                if(once){
                std::cout<<"assigment: ";
                for(auto e: assign)
                    std::cout<<e<<" ";
                std::cout<<std::endl;
            }
                InsertionHeuristic heur(false);
                heur.set_use_assignment(true);
                Tours t = heur(i,perm,assign);
                double make = i.makespan(t);
                if(min_insert==-1 or make < min_insert)
                    min_insert = make;
            }
            once = false;    
        }
        while ( std::next_permutation(perm.begin(), perm.end()) );

        if( std::round(opt) < std::round(min_insert)){
            std::cout<< "found instance without optimal insert order after "<<(seed+1)<<" runs" <<":"<<std::endl;
            std::cout<< i<<std::endl;
            std::cout << "opt tour: "<<opt_tour<< std::endl;
            std::cout << "opt: "<<opt<<std::endl;
            std::cout << "best insert: "<<min_insert<< std::endl;
            break;
        }
        ++seed;
        if(seed%100==0 and seed>0)
            std::cout<< "run: "<<seed<<std::endl;
    }

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
            ("lkh","Use the LKH heuristic to solve he tsp.")
            ("verbosity,v",po::value<int>(&verbosity)->default_value(0));

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style( po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")){
            cout<<"Calculates a lower bound for the given file.";
            cout<<"The bound is given by a TSP formulation, which is solved"
                   " optimally via Concorde.\n"; 
            cout<<boolalpha << desc << "\n";
            cout<<"required: 'f'"<<endl;
            return;
        }
        po::notify(vm); 
        
        //after parsing, execute the selected method    
        SingleCraneTSP_Solver solver;
        Instance i(filename);
        solver.set_verbosity(verbosity);
        solver.use_lkh(vm.count("lkh"));
        if(vm.count("localsearch"))
            solver.set_local_search(true);
       
        auto start = std::chrono::system_clock::now();    
	    auto tsp_result = solver(i);
	    auto stop = std::chrono::system_clock::now();
        auto runningtime = std::chrono::duration_cast<std::chrono::seconds>(stop - start);

	    Tours t{0};
        double bound;
        std::tie(bound,t) = solver(i);

        cout<<"makespan of a tour based on optimal single vehicle tour: ";
        cout<<i.makespan(t)<<endl;
        cout<<"TSP bound: "<<bound<<endl;
        cout<<"ratio: "<<i.makespan(t)*100/bound<<"%"<<endl;
        if (i.makespan(t)/bound > i.num_vehicles()){
             cerr<<"WARNING: ratio is above number of vehicles(";
             cerr<<i.num_vehicles()<<")"<<endl; 
        }  
        cout<<"Running time: "
            <<duration_to_string(runningtime) <<endl;

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
    if (argv.size()<2 or (argv.size() >0 and (argv[0]=="h" or argv[0]=="help"))){
		cout<<"batch \"<command> #1 <param> #2 <param> #1...\" <replace1>"
                " <replace2>... \n\tRuns the command in batch mode."<<endl;
		
        cout<<"Examples: "<<endl;
        cout<<"2DVS batch \"single_tsp -f #1 --lkh\" \"../data/generated/*/*.2dvs\n";
        cout<<"\t solves the single tsp for all files in all folder below generated.";
        cout<<"2DVS batch \"insert --k  #2 --n #3 -s #1\" 1  2-3  10,20,30\n";
        cout<<"\t runs the insertion heur. on files generated with 2 and 3 vehicles,\n";
        cout<<"\t 10,20 and 30 jobs using 1 as seed for the job generation."<<endl;
        return;
	}
	
	string command = argv[0];
	vector<vector<string>> replace_lists;
	
    //build intervals for numbers or lists of files, using wildcard '*'			
 	for(uint i=1; i< argv.size(); ++i){
        auto interval = create_interval(argv[i]);
        if(!interval.empty() and argv[i].find("*")==std::string::npos)
            replace_lists.push_back(interval);
	    else{ //try to find wildcard filelist
	        auto files =  find_files(argv[i]);
            std::sort(begin(files),end(files));
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



/* Helper function for the binary search*/
Tours test_makespan(Instance &i, double makespan, bool cuts){

    auto  mip = independent_TSP_MIP(i);
    mip.set_debug( false );
    mip.set_collision( true );
    mip.set_LP( false );
    mip.set_fixed_makespan(makespan);
    mip.set_silent(true);
    mip.use_subtour_cuts(cuts);
    return get<0>(mip.solve());
}

void binsearch(vector<string> argv){
	try{
        //define all parameters to set
        int verbosity;
        double epsilon = .5;
        double LB, UB;
        uint runs;
        int timelimit;
        string filename = "";


        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("filename,f", po::value<string>(&filename)->required(), 
                "2DVS file")
            ("runs,r",po::value<uint>(&runs)->default_value(10),
                "Number of runs used in the first step"
                " to find a heuristic solution.")
            ("LB,L",po::value<double>(&LB)->default_value(0),
                "uses the given lower bound for the binary search."
                " No check for validity performed!")
            ("UB,U",po::value<double>(&UB)->default_value(0),
                "uses the given upper bound for the binary search."
                " No check for validity performed!")
            ("cuts,C","enable subtour cuts as as user cuts, this is used "
                "per default in the normal MIP model.")
            ("timelimit,t",po::value<int>(&timelimit)->default_value(-1),
                "timelimit in minutes for the binary search."
                " A negative value indicates that there is no limit at all.")
            ("verbosity,v",po::value<int>(&verbosity)->default_value(0));

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style( po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")){
            cout<<"Calculates a lower bound for the given file.\n";
            cout<<"Then find a heuristic solution to the instance.\n";
            cout<<"With these two bound, a MIP is set up to close the\n" 
                  "gap between the best solution known and the lower bound.\n";
            cout<<"The MIP check finds a solution with a fixed makespan OR \n"
                   "there is no solution with that particular makespan.\n";
            cout<<" In both cases a binary search is performed"
                    " to tighten the gap.";
            cout<<boolalpha << desc << "\n";
            cout<<"required: 'f'"<<endl;
            return;
        }
        po::notify(vm); 
        

        //after parsing, execute the selected method    
       
        Instance i(filename);
      
        bool add_cuts = vm.count("cuts") > 0;
        //find valid LB if no value set
        if(LB==0){
            SingleCraneTSP_Solver solver;
            solver.set_verbosity(verbosity);
            solver.set_local_search(false);
            auto tsp_result = solver(i,true);
            LB = std::get<0>(tsp_result);
        }

        //find valid UB if no value set
        if(UB==0){
            InsertionHeuristic heur(true);
            heur.set_verbosity(verbosity);
            heur.set_use_assignment(true);
            heur.set_runs(runs);
            Tours t = heur(i);
            UB = i.makespan(t);
        }

        cout<< "Starting binary seach using bounds:\n";
        cout<< " lb - sol. \t\t gap\n";
        cout<< std::fixed << std::setprecision(2); //only two digits for the gap
        cout<< " "<<LB <<" - "<< UB<<"\t\t"<<(UB/LB-1)*100<<"%"<<endl;
        if(timelimit > 0)
           cout<<" timelimit: "<<minutes_to_string(timelimit)<<endl;

        if(UB < LB){
            cerr<<"Lower bound("<<LB<<") is bigger than upper bound("<<UB<<")";
            cerr<<" Abort!" <<endl;
            return;
        }

        //start the binary search here!
        auto starttime = std::chrono::system_clock::now();
        bool time_left = true;
        while(time_left and fabs(UB-LB) > epsilon){
            int to_test = (UB+LB)/2.0;
            auto sol = test_makespan(i,to_test,add_cuts);
            if(verbosity > 0){ cout<<"result for "<<to_test<<endl;
                if(sol.empty())
                    cout<< "no solution "<<endl;
                else cout<< "new makespan: "<<i.makespan(sol)<<endl;
            }    
            if( sol.empty() ) LB = to_test+1;
            else              UB = i.makespan(sol); 

            cout<< " "<<LB <<" - "<< UB<<"\t\t"<< (1-LB/UB)*100<<"%"<<endl;
            auto now = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<minutes>(now - starttime);
            if(timelimit > 0)
                time_left = (elapsed.count() < timelimit); 
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

struct sol_function{
    sol_function(std::string const& val): value(val){ }
    std::string value;

    static vector<std::string> allowed_values;

    Tours generate_Tour(const Instance & i) const{

        if(value == "TSP" or value == "TSP_ls"){
            SingleCraneTSP_Solver solver;
            solver.set_verbosity(0);
            solver.set_local_search(value == "TSP_ls");
           return std::get<1>(solver(i)); 
        }else if(value == "insertion" or value == "insertion_ls"){
            InsertionHeuristic solver(value == "insertion_ls");
            solver.set_verbosity(0);
            solver.set_stop_at_better(true);
            solver.set_use_assignment(true);

            return solver(i); 
        }

        return Tours{i.num_vehicles()};
    }
};

vector<std::string> sol_function::allowed_values = {"TSP","TSP_ls","insertion",
                                                    "insertion_ls","none"};



void validate(boost::any& v, 
              std::vector<std::string> const& values,
              sol_function* /* target_type */,
              int)
{
    using namespace boost::program_options;
    // Make sure no previous assignment to 'v' was made.
    validators::check_first_occurrence(v);

    // Extract the first string from 'values'. If there is more than
    // one string, it's an error, and exception will be thrown.
    string const& s = validators::get_single_string(values);
    vector<std::string> & vec = sol_function::allowed_values;

    if ( std::find(begin(vec), end(vec), s)!=end(vec)){
        v = boost::any(sol_function(s));
    } else {
        //add allowed values
        throw boost::program_options::invalid_option_value(s);
    }
}


void insertion_mip(std::vector<std::string> argv){
try{
        //define all parameters to set
        int verbosity;
        uint clustersize;
        string filename = "";
        int timelimit;  
        uint seed;
        sol_function sol{"none"};

        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("filename,f", po::value<string>(&filename)->required(), 
                "2DVS file")
            ("clustersize,c",po::value<uint>(&clustersize)->default_value(8),
                " number of jobs that will be considered and "
                 "solved to optimility.")
            ("localsearch,l", " use a local search to find better solutions")
            ("seed,s",po::value<uint>(&seed)->default_value(0),
                " seed used to find a random permutation to start with.")
            ("inital_solver,i", po::value<sol_function>(&sol), ("set initial solver function\n\t"
                "possible values: "+
                to_str(sol_function::allowed_values)).c_str())
            ("timelimit,t",po::value<int>(&timelimit)->default_value(-1),
                "timelimit in minutes for the binary search."
                " A negative value indicates that there is no limit at all.")
            ("verbosity,v",po::value<int>(&verbosity)->default_value(0));


        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style( po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")){
            cout<<"Splits the instance into several smaller ones.\n";
            cout<<"The size of the smaller problems is given by the clustersize.\n";
            cout<<" Every instance is solved optimally via MIP and a solution is "
            "build by adding the jobs into a solution for all jobs.";
            cout<<boolalpha << desc << "\n";
            cout<<"required: 'f'"<<endl;
            return;
        }
        po::notify(vm); 
        

        Instance i(filename);
      
        bool localsearch = vm.count("localsearch") > 0;
        MipInsertionHeuristic insert(localsearch);
        insert.set_verbosity(verbosity);
        insert.set_seed(seed);
        insert.set_cluster_size(clustersize);
        insert.set_timelimit(timelimit);


        Tours t(i.num_vehicles());
        if(vm.count("inital_solver")>0){
            auto perm = sol.generate_Tour(i).startingtime_permutation();
            //auto perm = sol.generate_Tour(i).endingtime_permutation();
            t = insert(i,perm);
        } else {
            t = insert(i);
        }
       
        cout<< "Found solution with makespan "<<i.makespan(t)<<endl;
        if(verbosity>0)
            cout<<t<<endl;

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

void separate(std::vector<std::string> argv){
    try {
        //define all parameters to set
        int verbosity;
        int k,n; 
        uint seed;
        string filename = "";

        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("filename,f", po::value<string>(&filename), 
             "2DVS file")
            ("k", po::value<int>(&k), 
             "set number of vehicles/cranes")
            ("n", po::value<int>(&n), 
             "set number of jobs to generate in the instance")
            ("seed,s", po::value<unsigned int>(&seed)->default_value(0), 
              "set seed for random samples")
            ("initial,i", "stop after initial solution")
            ("lkh,l","use LKH instead of concorde to solve the TSPs")
            ("verbosity,v",po::value<int>(&verbosity)->default_value(0));

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style( po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")){
            cout<<"Finds a solution for the given file using the";
            std::cout<<" separation heuristic.\n";
            cout<<boolalpha << desc << "\n";
            cout<<"required: 'f'"<<endl;
            return;
        }
        po::notify(vm); 
        
        //after parsing, execute the selected method
        auto start = std::chrono::system_clock::now();    
        SeparationHeuristic heur;
        heur.only_initial( vm.count("initial")>0 );
        heur.use_lkh( vm.count("lkh")>0 );
        Instance i; 
        if(vm.count("filename")){
            i = Instance{filename};
            if( vm.count("n")) 
                cout<<"Warning: Ignoring given number of jobs!"<<endl;
            if( vm.count("k")){
                cout<<"Warning: Ignoring number of vehicles in file!"<<endl;
                i.set_num_vehicles(k);
            }
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

        heur.set_verbosity(verbosity);
        Tours t = heur(i);
        auto stop = std::chrono::system_clock::now();
        auto runningtime = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        cout<<"makespan of a separation heur. solution: ";
        cout<<i.makespan(t)<<endl;
        
        cout<<"Running time: "
            <<duration_to_string(runningtime) <<endl;
        if(not i.verify(t))
            cout<<"WARNING: Solution not valid!"<<endl;


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



void consolidate(std::vector<std::string> argv){
    try {
        //define all parameters to set
        int verbosity;
        int k,n; 
        uint seed;
        string filename = "";

        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("filename,f", po::value<string>(&filename), 
             "2DVS file")
            ("k", po::value<int>(&k), 
             "set number of vehicles/cranes")
            ("n", po::value<int>(&n), 
             "set number of jobs to generate in the instance")
            ("localsearch,l", "use a local search on the created permutation and assigment.")
            ("seed,s", po::value<unsigned int>(&seed)->default_value(0), 
              "set seed for random samples")
            ("verbosity,v",po::value<int>(&verbosity)->default_value(0));

        po::variables_map vm;        
        po::store(po::command_line_parser(argv)
                    .options(desc)
                    .style( po::command_line_style::unix_style)
                    .run(), vm); 

        //react on som settings
        if (vm.count("help")>0){
            cout<<"Finds a solution for the given file using the";
            std::cout<<" consolidation heuristic.\n";
            cout<<boolalpha << desc << "\n";
            cout<<"required: 'f'"<<endl;
            return;
        }
        po::notify(vm); 
        
        //after parsing, execute the selected method
        auto start = std::chrono::system_clock::now();    
        ConsolidationHeuristic heur;
        heur.use_local_search( vm.count("localsearch")>0 );
        Instance i; 
        if(vm.count("filename")>0){
            i = Instance{filename};
            if( vm.count("n")>0) 
                cout<<"Warning: Ignoring given number of jobs!"<<endl;
            if( vm.count("k")>0){
                cout<<"Warning: Ignoring number of vehicles in file!"<<endl;
                i.set_num_vehicles(k);
            }
        }else{
            if( vm.count("k")==0 and vm.count("n")==0 ) {
                cout<< "Number of jobs and vehicles not given."
                      << " No filename given. Usage:\n"<<desc<<endl;
                return;
            }
            i.set_num_vehicles(k);
            i.generate_random_depots(0,100,0,20,seed);
            i.generate_random_jobs(n,0,100,0,20,seed); 
        }

        heur.set_verbosity(verbosity);
        Tours t = heur(i);
        auto stop = std::chrono::system_clock::now();
        auto runningtime = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        cout<<"makespan of a separation heur. solution: ";
        cout<<i.makespan(t)<<endl;
        
        cout<<"Running time: "
            <<duration_to_string(runningtime) <<endl;
        if(not i.verify(t))
            cout<<"WARNING: Solution not valid!"<<endl;


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