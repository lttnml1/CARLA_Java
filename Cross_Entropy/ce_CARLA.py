#!/usr/bin/env python

#**********************************************************************
#   Author: Matthew Litton
#   Last Modified: 9/8/2022
#   Purpose: Cross-Entropy simulation for CARLA
#**********************************************************************

"""
Outline of program

1) Initialize parameters (N, rho, gamma)

2) Run cross-entropy while(g>gamma)
    2a) draw N random samples
    2b) score each random sample
    2c) sort from smallest to largest
    2d) find the elite set (g is the largest in the elite set)
    2e) update the parameters based on the unscored samples that make up the elite set

3) now you have a distribution that is more likely to generate "bad" behavior

"""


import time

def main():
    program_start_time = time.time()        
            

    print(f"TOTAL RUN TIME: {time.time()-program_start_time}")
if __name__ == '__main__':
    main()
