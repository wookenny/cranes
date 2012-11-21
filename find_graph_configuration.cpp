#include <ilcplex/ilocplex.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <set>
#include <tuple>
#include <array>
#include <cmath>
#include <vector>

using namespace std;

ILOSTLBEGIN

//TODO: change mapping from string->int to string->IloNumVar
//TODO: do I need name_var(i,j)?

class stringify{
	string s_;
	public:
		stringify(const string s):s_(s){}
		string operator()(int i,int j) const {
				return s_+"_"+to_string(i)+"_"+to_string(j);}
		string operator()(int i) const {return s_+"_"+to_string(i);}
};

class variable_access{
	private:
		stringify& name_;
		IloNumVarArray& vars_;
		unordered_map<string,int>& pos_;
	public: 
		variable_access(stringify& name,
						IloNumVarArray& vars,
						unordered_map<string,int>& pos):
						name_(name),vars_(vars),pos_(pos){}
		IloNumVar& operator()(int i){
				return vars_[pos_[name_(i)]]; 
		}
		IloNumVar& operator()(int i,int j){
				return vars_[pos_[name_(i,j)]]; 
		}
};

stringify name_t("t");  
stringify name_x("x");  
stringify name_y("y");  
stringify name_e1("e1");  
stringify name_e2("e2");
stringify name_n1("n1");
stringify name_n2("n2");
stringify name_a1("a1");
stringify name_a2("a2");  
stringify name_z1("z1");
stringify name_z2("z2");


class graph{
 	private:
 	set<tuple<int,int>> _edges;
	set<tuple<int,int>> _arcs;
	
	public:
	graph(){}
	void add_edge(int i,int j){ 
		_edges.insert(make_tuple(i,j));  
		_edges.insert(make_tuple(j,i));
	}
	
	void add_arc(int i,int j){ 
		_arcs.insert(make_tuple(i,j));  
	}	
	
	bool is_edge(int i, int j) const{ return _edges.find(make_tuple(i,j))!=_edges.end();}
	//if is_arc(i,j) is true, is_arc(j,i ) is false
	//IMORTANT: is_no_edge() is false for (i,j) AND (j,i)
 	bool is_arc(int i, int j) const{ return _arcs.find(make_tuple(i,j))!=_arcs.end(); }
	bool is_no_edge(int i, int j) const{
		return !is_edge(i,j) && !is_arc(i,j) && !is_arc(j,i);
	}
	
	bool edge_correct(array<double,3> i, array<double,3> j) const{
		double x1, y1, t1, x2, y2, t2;
		tie(x1,y1,t1) = make_tuple(i[0],i[1],i[2]);
		tie(x2,y2,t2) = make_tuple(j[0],j[1],j[2]);
	
		return (fabs(x1-x2) <= fabs(t1-t2)) and (fabs(t1-t2) < fabs(y1-y2));
	}
	
	bool arc_correct(array<double,3> i, array<double,3> j) const{
		double x1, t1, x2, t2;
		tie(x1,ignore,t1) = make_tuple(i[0],i[1],i[2]);
		tie(x2,ignore,t2) = make_tuple(j[0],j[1],j[2]);
		
		return (x1<x2) and (fabs(x1-x2) > fabs(t1-t2));
	}

	bool no_edge_correct(array<double,3> i, array<double,3> j) const{
		double x1, y1, t1, x2, y2, t2;
		tie(x1,y1,t1) = make_tuple(i[0],i[1],i[2]);
		tie(x2,y2,t2) = make_tuple(j[0],j[1],j[2]);
		
		return (fabs(x1-x2) <= fabs(t1-t2)) and (fabs(t1-t2) >= fabs(y1-y2));
	}	
	
	bool test_solution( vector<array<double,3>> coords) const{
		for(uint i=0; i<coords.size(); ++i)
			for(uint j=0; j<coords.size(); ++j){
				if(i==j) continue;
				if( is_edge(i+1,j+1) && not edge_correct(coords[i],coords[j]))
					return false;
				if( is_arc(i+1,j+1) &&  not arc_correct(coords[i],coords[j]))
					return false;
				if( is_no_edge(i+1,j+1) &&  not no_edge_correct(coords[i],coords[j]))
					return false;	
			
				return true;
			}
		return true;
	}

};


vector<array<double,3>> solveLP(int size, graph g){
	
	double EPS = 1;
	int bigM = 200;
	auto type = IloNumVar::Bool;
	vector<array<double,3>> result;
	
	
	IloEnv env;
	try {
		IloModel model(env);
		IloNumVarArray vars(env);
		IloConstraintArray cons(env);
		
		unordered_map<string,int> v;
		int counter = 0;
		
		//setup easy access to variables!
		variable_access x(name_x, vars, v);
		variable_access t(name_t, vars, v);
		variable_access y(name_y, vars, v);
		variable_access e1(name_e1, vars, v);
		variable_access e2(name_e2, vars, v);
		variable_access n1(name_n1, vars, v);
		variable_access n2(name_n2, vars, v);
		variable_access a1(name_a1, vars, v);
		variable_access a2(name_a2, vars, v);
		variable_access z1(name_z1, vars, v);
		variable_access z2(name_z2, vars, v);
		
		//variables: lower and upper bound and the name! 
		//add varibles
		for(int i=1;  i<= size; ++i){
			//t
			v[name_t(i)] = counter++;
			vars.add( IloNumVar(env,  0, bigM,  IloNumVar::Float, name_t(i).c_str() ) );	
			//x
			v[name_x(i)] = counter++;
			vars.add( IloNumVar(env,  0, bigM,  IloNumVar::Float, name_x(i).c_str() ) );
			//y
			v[name_y(i)] = counter++;
			vars.add( IloNumVar(env,  0, bigM, IloNumVar::Float, name_y(i).c_str() ) );
			for(int j=1; j<=size; ++j){
				if(i==j) continue;
				//e1
				v[name_e1(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_e1(i,j).c_str() ) );
				//e2
				v[name_e2(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_e2(i,j).c_str() ) );
				//n1
				v[name_n1(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_n1(i,j).c_str() ) );
				//n2
				v[name_n2(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_n2(i,j).c_str() ) );
				//a1
				v[name_a1(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_a1(i,j).c_str() ) );
				//a2
				v[name_a2(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_a2(i,j).c_str() ) );
				
				//z1
				v[name_z1(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_z1(i,j).c_str() ) );
				//z2
				v[name_z2(i,j)] = counter++;
				vars.add( IloNumVar(env,  0, 1, type, name_z2(i,j).c_str() ) );
			}
		}
		
		//add all constraints
		model.add(vars);		
				
		//subject: feasibility
		model.add(IloMaximize(env, 0));
		
		
		//add constraints
		//add constraints for arcs
		for(int i=1;  i<= size; ++i){
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				if(g.is_arc(i,j)){
					cout<< "constr. for arc ("<<i<<":"<<j<<")" <<endl;
					//xi < xj
					cons.add( x(i)  <= x(j) - EPS );
					
					//|ti-tj| < |xi - xj|
					cons.add(  t(j)  - t(i)  <= x(j) - x(i) + (1-a1(i,j))*bigM -EPS);		
					cons.add( t(i)  - t(j)  <= x(j) - x(i) + (1-a1(i,j))*bigM -EPS);
					cons.add( t(j)  - t(i)  <= x(i) - x(j) + (1-a2(i,j))*bigM -EPS);		
					cons.add( t(i)  - t(j)  <= x(i) - x(j) + (1-a2(i,j))*bigM -EPS);	
										
					cons.add( a1(i,j) + a2(i,j) >= 1 );
					
				}
				
			}
		}
		
		//constraints for edges
		for(int i=1;  i<= size; ++i){
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				if (g.is_edge(i,j)){
					cout<< "constr. for  edge ("<<i<<":"<<j<<")" <<endl;
					// |t1-t2| < |y1-y2|
					cons.add( t(j)  - t(i)  <= y(j) - y(i) + (1-e1(i,j))*bigM -EPS);
					cons.add( t(i)  - t(j)  <= y(j) - y(i) + (1-e1(i,j))*bigM -EPS);
					cons.add( t(j)  - t(i)  <= y(i) - y(j) + (1-e2(i,j))*bigM -EPS);
					cons.add( t(i)  - t(j)  <= y(i) - y(j) + (1-e2(i,j))*bigM -EPS);
					cons.add( e1(i,j) + e2(i,j) >= 1 );
			
					//|x1-x2| <= |t1-t2| 	
					cons.add( x(j)  - x(i)  <= t(j) - t(i) + (1-z1(i,j))*bigM );
					cons.add( x(i)  - x(j)  <= t(j)  - t(i) + (1-z1(i,j))*bigM);
					cons.add( x(j)  - x(i)  <= t(i)  - t(j) + (1-z2(i,j))*bigM);
					cons.add( x(i)  - x(j)  <= t(i)  - t(j) + (1-z2(i,j))*bigM);
					
					cons.add( z1(i,j) + z2(i,j) >= 1 );
				}
			}
		}


		//constraints for non-existing edges
		for(int i=1;  i<= size; ++i){
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				if (g.is_no_edge(i,j)){
					cout<< "constr. for non_edge ("<<i<<":"<<j<<")" <<endl;
					// |y1-y2| <= |t1-t2|
					cons.add( y(j)  - y(i)  <= t(j) - t(i) + (1-n1(i,j))*bigM);
					cons.add( y(i)  - y(j)  <= t(j) - t(i) + (1-n1(i,j))*bigM);
					cons.add( y(j)  - y(i)  <= t(i) - t(j) + (1-n2(i,j))*bigM);
					cons.add( y(i)  - y(j)  <= t(i) - t(j) + (1-n2(i,j))*bigM);
					
					cons.add( n1(i,j) + n2(i,j) >= 1 );
			
			
					//|x1-x2| <= |t1-t2| 		
					cons.add(  x(j)  - x(i)  <= t(j) - t(i) + (1-z1(i,j))*bigM );
					cons.add( x(i)  - x(j)  <= t(j) - t(i) + (1-z1(i,j))*bigM);
					cons.add( x(j)  - x(i)  <= t(i) - t(j) + (1-z2(i,j))*bigM);
					cons.add( x(i)  - x(j)  <= t(i) - t(j) + (1-z2(i,j))*bigM);
					
					cons.add( z1(i,j) + z2(i,j) >= 1 );
				}
			}
		}
		
		//add all constraints to the model
		model.add(cons);
		
		
		//solve the model
		IloCplex cplex(model);
		cplex.extract(model);
		cout<< model<<endl;
		
		//solve silently
		//cplex.setOut(env.getNullStream());
		bool solved = cplex.solve();
		if ( !solved ) {
			cerr<<"MIP model is infeasible!"<<endl;
			
			
			//conflict refiner
			IloConstraintArray infeas(env);
		    IloNumArray preferences(env);

			infeas.add(cons );
			
		     for (IloInt i = 0; i<vars.getSize(); i++) {
		        if ( vars[i].getType() != IloNumVar::Bool ) {
		          infeas.add(IloBound(vars[i], IloBound::Lower));
		          infeas.add(IloBound(vars[i], IloBound::Upper));
		        }
		     }

		     for (IloInt i = 0; i<infeas.getSize(); i++) {
		       preferences.add(1.0);  // user may wish to assign unique preferences
		     }

		     if ( cplex.refineConflict(infeas, preferences) ) {
		      IloCplex::ConflictStatusArray conflict = cplex.getConflict(infeas);
		        env.getImpl()->useDetailedDisplay(IloTrue);
		        cout << "Conflict :" << endl;
		        for (IloInt i = 0; i<infeas.getSize(); i++) {
		          if ( conflict[i] == IloCplex::ConflictMember)
		               cout << "Proved  : " << infeas[i] << endl;
		          if ( conflict[i] == IloCplex::ConflictPossibleMember)
		               cout << "Possible: " << infeas[i] << endl;
		        }
		     }
		     else
		        cout << "Conflict could not be refined" << endl;
         cout << endl;
			
			
			env.end();
			return result;
		}
		
		for(int i=1;  i<= size; ++i){
			array<double,3> coord = {{cplex.getValue(x(i)),
									  cplex.getValue(y(i)),
						   			  cplex.getValue(t(i))}}; 
			result.push_back(coord);
		}
		
		//print variables
		for(int i=1;  i<= size; ++i){
			cout<< name_x(i) << " = "<<	cplex.getValue(x(i));
			cout<<"\t\t"<<name_y(i) << " = "<<	cplex.getValue(y(i));
			cout<<"\t\t"<<name_t(i) << " = "<<	cplex.getValue(t(i))<<endl;			
		}
		cout<<endl;
		for(int i=1;  i<= size; ++i)
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				cout<< name_z1(i,j) << " = "<<	cplex.getValue(z1(i,j));
				cout<<"\t"<<name_z2(i,j) << " = "<<	cplex.getValue(z2(i,j))<<endl;			
			}	
		for(int i=1;  i<= size; ++i)
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				if (not g.is_edge(i,j)) continue;
				cout<< name_e1(i,j) << " = "<<	cplex.getValue(e1(i,j));
				cout<<"\t"<<name_e2(i,j) << " = "<<	cplex.getValue(e2(i,j))<<endl;			
			}
		for(int i=1;  i<= size; ++i)
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				if (g.is_edge(i,j)) continue;
				cout<< name_n1(i,j) << " = "<<	cplex.getValue(n1(i,j));
				cout<<"\t"<<name_n2(i,j) << " = "<<	cplex.getValue(n2(i,j))<<endl;			
			}
		for(int i=1;  i<= size; ++i)
			for(int j=1;  j<= size; ++j){
				if(i==j) continue;
				if (not g.is_arc(i,j)) continue;
				cout<< name_a1(i,j) << " = "<<	cplex.getValue(a1(i,j));
				cout<<"\t"<<name_a2(i,j) << " = "<<	cplex.getValue(a2(i,j))<<endl;			
			}
	}
	catch (IloException& e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught" << endl;
	}
	env.end();
	return result;
}


int main ()
{

	
	int size = 6;	
	graph g;
	g.add_arc(1,2); g.add_arc(2,3); g.add_arc(1,3);
	g.add_arc(4,5); g.add_arc(5,6); g.add_arc(4,6);
	//g.add_arc(7,8); g.add_arc(8,9); g.add_arc(7,9);
	

	g.add_edge(1,5); g.add_edge(2,4); g.add_edge(3,6); 
	
	//g.add_edge(4,2);
	//g.add_edge(1,3); g.add_edge(3,5); g.add_edge(1,5);	 
	
	auto sol = solveLP(size,g);
	cout<< boolalpha;
	if(sol.size()!=0)
		cout<< "solution correct: "<< g.test_solution(sol)<<endl;
	else
		cout<< "solution correct: Not tested since no solution was found!"<<endl; 
	
	return 0;
}




