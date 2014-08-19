#!/usr/bin/env python

import os, re, csv

def convert_to_seconds(time):
    m = re.search('([0-9]*)[^0-9]*([0-9]*)',time.strip())
    if m==None:
        assert(False)
    if m.group(2)=='':
        return int(m.group(1))
    else:
        res = int(m.group(1))*60+int(m.group(2))           
        #print time,'->',res        
        return res
        
def parse_results(fname):
    res = {}
    with open(fname) as f:
        content = f.readlines()
    #content = content.split("\n")
    
    datafile = None
    makespan = None
    bound    = None
    
    for c in content:
        #jump over lines like: 
        #'--------'
        if c.startswith('-------') and bound==None:
            continue
        
        if c.startswith('-------') and bound!=None:
            res[datafile] = (bound)
            datafile = bound = None
            
        #read file and call from lines like
        #calling: consolidate -f ../data/generated/L1_d0_k1/L1_d0_k1_0.2dvs 
        if c.startswith('calling: '):
            match = match = re.match( r'calling: [^\.]*(.*2dvs).*', c)
            datafile = match.group(1).split("/")[-1]
        
        #read makespan from lines like
        #makespan of a separation heur. solution: 28532
        if c.startswith('Objective') or c.startswith("TSP bound"):
            bound = float(c.split(':')[-1])
    
        #if "makespan" in c:
        #    makespan = int(c.split(':')[-1])

    if(bound!=None):
        res[datafile] = (bound)
        datafile = bound = None
    return res                
            
def collect_files():
    return ['results_bounds_LPrelax.txt', 'results_bounds_TSP_LKH_nolocalsearch.txt']
    
    
files = collect_files()
print 'using ',files
methods = []
for f in files:    
    methods.append(parse_results(f))
datafile = sorted(methods[0].keys())
   
with open('bounds.csv', 'wb') as csvfile:
    writer = csv.writer(csvfile, delimiter=';',
                            quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #write methods
    header = ['filename']
    for f in files:
        header.extend([f, ''])
    writer.writerow(header)
    #write row itself
    for d in datafile:
        row = [d]
        for m in methods:
            row.append(m[d])
        if 'd75' in d:
            writer.writerow(row)
    
            
