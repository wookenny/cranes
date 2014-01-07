import os

basefolder = '../data/generated/'

length = {}
length['L1'] = (0,100,0,100) #whole length
length['L2'] =(0,10,0,100) #short jobs
length['L3'] = (75,100,0,50) #long jobs
length['L4'] = (40,60,40,60) #all middle length   

num_vehicles = {'k1':1,'k2':2,'k3':3, 'k4':4, 'k5':5}
n = 100 #number of jobs

driveby = {'d0':0, 'd50':50, 'd75':75} # percantage of driveby jobs

#boundary settings
min_x = 0
max_x = 500
min_y = 0
max_y = 50

def create_call(l,k,d,i): 
    call = './2DVS write -v 0 -n %d -s %d -x %d -X %d -y %d -Y %d'%(n,i,min_x,max_x,min_y,max_y)
    call += ' -k %d -d %d'%(num_vehicles[k],driveby[d])
    call += ' --minlengthx %d --maxlengthx %d --minlengthy %d --maxlengthy %d'%length[l] 
    call += ' -f %s/%s.2dvs'%(basefolder+l+'_'+d+'_'+k,l+'_'+d+'_'+k+'_'+str(i))
    return call

#number of jobs per setting
jobs_per_setting = 10
for l in length:
    for k in num_vehicles:
        for d in driveby:
            folder = basefolder+l+'_'+d+'_'+k
            if not os.path.exists(folder):
                os.mkdir(folder)
            for i in range(jobs_per_setting):
                call = create_call(l,k,d,i)
                #print call
                os.system(call)    
