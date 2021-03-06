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
    time     = None
    makespan = None
    call     = None
    
    for c in content:
        #jump over lines like: 
        #'--------'
        if c.startswith('-------') and makespan==None:
            continue
        
        if c.startswith('-------') and makespan!=None:
            res[datafile] = (makespan,time)
            datafile = makespan = time = None
            
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
            time = convert_to_seconds(time)

    if(makespan!=None):
        res[datafile] = (makespan,time)
        datafile = makespan = time = None
    return res                
            
def collect_files():
    return [f for f in  os.listdir('.') if f.endswith('.txt') and f.startswith('results')  and not 'bound' in f ]
    
    
files = collect_files()
print 'using ',files
methods = []
for f in files:    
    methods.append(parse_results(f))
datafile = sorted(methods[0].keys())
   
with open('heuristics.csv', 'wb') as csvfile:
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
            row.extend(m[d])
        #if 'd75' in d:    
        writer.writerow(row)
    
            
