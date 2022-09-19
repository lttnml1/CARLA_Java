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
import numpy as np
import argparse


from cross_entropy import CrossEntropy
from normal_distrib import NormalDistrib

def main():
    program_start_time = time.time()

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2004,
        type=int,
        help='TCP port to listen to (default: 2004)')
    argparser.add_argument(
        '--no_render',
        action = 'store_true',
        help='Render graphics (default: False)')
    args = argparser.parse_args()

    try:
        
        distributions = []
        adversary_target_speed = NormalDistrib(2,1)
        distributions.append(adversary_target_speed)
        
        ce = CrossEntropy(10,0.1,5,distributions)
        ce.execute_ce(args)

        #this is manual - in the future, this will automatically update
        #ce.distributions[0].mu = 2.6401866440899724
        #ce.distributions[0].sigma = 0.08131476220951864
        
        ans = input("CE search is done, continue with demonstrate and label? y/n: ")
        if(ans == 'y'):
            ce.demonstrate_and_label(args, 1)
        
    
    finally:
        print(f"TOTAL RUN TIME: {time.time()-program_start_time}")

if __name__ == '__main__':
    main()
