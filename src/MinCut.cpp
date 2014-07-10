#include "MinCut.h"


#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/typeof/typeof.hpp>

struct edge_t
{
  unsigned int first;
  unsigned int second;
};

std::vector<bool> find_min_cut(const std::vector<std::tuple<int,int,double>> &edges,
                                uint n)
{
   
    using namespace std;
  
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
     boost::no_property, boost::property<boost::edge_weight_t, double> > undirected_graph;
    typedef boost::property_map<undirected_graph, boost::edge_weight_t>::type weight_map_type;
    typedef boost::property_traits<weight_map_type>::value_type weight_type;
  
    // define the 16 edges of the graph. {3, 4} means an undirected edge between vertices 3 and 4.
    edge_t *E= new edge_t[edges.size()];

    // for each of the 16 edges, define the associated edge weight. ws[i] is the weight for the edge
    // that is described by edges[i].
    weight_type *ws = new weight_type[edges.size()];

    unsigned int u,v;
    double capacity;
    int i = 0;
    for(auto e: edges){
       std::tie(u,v,capacity) = e;
       //std::cout<<u<<"->"<<v <<std::endl;
       assert(u<n);
       assert(u>=0);
       assert(v<n);
       assert(v>=0);
       assert(capacity >=0);
       edge_t edge;
       edge.first = u;
       edge.second = v;
       E[i] = edge;
       ws[i] = capacity; 
       ++i;
    }   

    //find the cut
  
    // construct the graph object. n is the number of vertices, 
    //which are numbered from 0 through n-1, and  is the number of edges.
  
    undirected_graph g(E, E+edges.size(), ws, n, edges.size());

     // define a property map, `parities`, that will store a boolean value for each vertex.
    // Vertices that have the same parity after `stoer_wagner_min_cut` runs are on the same side of the min-cut.
    BOOST_AUTO(parities, boost::make_one_bit_color_map(num_vertices(g),
                         get(boost::vertex_index, g)));

    // run the Stoer-Wagner algorithm to obtain the min-cut weight. `parities` is also filled in.
    double w = boost::stoer_wagner_min_cut(g, get(boost::edge_weight, g), boost::parity_map(parities));
    assert(num_vertices(g)==n);
    //return the correct set, if cut value >2, return empty set!
    std::vector<bool> cutSet;
    if(w < 2-0.001){
        for (size_t i = 0; i < num_vertices(g); ++i){
            cutSet.push_back( get(parities, i) );
        }
    }
    
    delete[] ws;
    delete[] E;
    return cutSet;
}


