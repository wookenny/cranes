#!/usr/bin/env python

import os, re




def parse_results(fname):
    with open(fname) as f:
        content = f.readlines()
    #content = content.split("\n")
    
    datafile = None
    time     = None
    makespan = None
    #call     = None
    
    for c in content:
        #jump over lines like: 
        #'--------'
        if c.startswith('-------'):
            continue
            
        #read file and call from lines like
        #calling: consolidate -f ../data/generated/L1_d0_k1/L1_d0_k1_0.2dvs 
        if c.startswith('calling: '):
            match = match = re.match( r'calling: [^\.]*(.*2dvs).*', c)
            datafile = match.group(1).split("/")[-1]
        
        #read makespan from lines like
        #makespan of a separation heur. solution: 28532
        if c.startswith('makespan '):
            makespan = int(c.split(':')[-1])

        #read runningtimes and print all while reading lines like
        #Running time: 6 seconds
        if c.startswith('Running time:'):
            time = c.split(':')[-1]
            print datafile,makespan,time,
            datafile = makespan = time = None
            
            
            
results = [f for f in  os.listdir('.') if f.endswith('.txt') and f.startswith('results')]
    
parse_results(results[0])
print results[0]     
            
            
