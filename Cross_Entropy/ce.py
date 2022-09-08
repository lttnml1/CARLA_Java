#!/usr/bin/env python

from time import time
import numpy as np
from numpy.random import default_rng
import time
import math


#set hyperparameters
u = [0.25,0.4,0.1,0.3,0.2]
v=u
gamma = 2
N=10**3
N1= 10**5
n = len(u)
rho = 0.1


#initialize rng
rng = default_rng()

def S_len(X):
    s1 = np.minimum(X[:,0]+X[:,3],X[:,0]+X[:,2]+X[:,4])
    s2 = np.minimum(X[:,1]+X[:,4],X[:,1]+X[:,2]+X[:,3])
    s = np.minimum(s1,s2)
    return s

def main():
    program_start_time = time.time()
    
    g=0
    while(g<gamma):
        y=np.empty((N,n))
        for i in range(n):
            y_i = rng.exponential(u[i],N)
            y[:,i] = y_i
        
        S = S_len(y)

        SS = np.sort(S)
        SSidx = np.argsort(S)

        eidx = round((1-rho)*N)
        g = SS[eidx]

        if g>=gamma:
            g = gamma
            while(SS[eidx] >= g):
                eidx-=1
            eidx+=1
        
        W = np.ones(N)
        for j in range(n):
            W = W * np.exp(-y[:,j] * (1/u[j] - 1/v[j])) * v[j]/u[j]
        
        for j in range(n):
            v[j] = np.sum(W[SSidx[eidx:N]] * y[SSidx[eidx:N],j]) / np.sum(W[SSidx[eidx:N]])
        
        print(f"{g:6.4f}:\t{v}\n")
            
            

    print(f"TOTAL RUN TIME: {time.time()-program_start_time}")

if __name__ == '__main__':
    main()
