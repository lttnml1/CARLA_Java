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
import os
import random


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
        
        ce = CrossEntropy(10,.1,5,distributions)
        #ce.execute_ce_good(args)
        #ce.execute_ce_bad(args)
        print(f"CE SEARCH RUN TIME: {time.time()-program_start_time}")

        #this is manual - in the future, this will automatically update
        ce.distributions[0].mu = 3.2841948426060585
        ce.distributions[0].sigma = 1
        
        
        ans = input("CE search is done, continue with demonstrate and label? y/n: ")
        if(ans == 'y'):
            ce.demonstrate_and_label(args, 5)
        
        
        """
        replay_path = "c:\\data\\label\\"
        #num_files = 20
        
        for dirName, subdirList, fileList in os.walk(replay_path):
            #for i in range(num_files):
            good_files = [f for f in fileList if '0_path' in f]
            bad_files = [f for f in fileList if '1_path' in f]
            for i in range(len(good_files)):
                ans = input(f"{i}: continue?")
                #file = random.choice([f for f in fileList if '0_path' in f])
                print(good_files[i])
                ret = ce.replay(args, os.path.join(dirName,good_files[i]))
                if ret < 0:
                    print("Replay cancelled by user!")
                    return
        
        
        """
        #good_files = ['20220923-124856_5_0_path.csv','20220923-125221_9_0_path.csv','20220923-125246_11_0_path.csv','20220923-132603_8_0_path.csv','20220923-132726_15_0_path.csv','20220928-110918_14_0_path.csv','20220928-110954_9_0_path.csv','20220928-111146_4_0_path.csv','20220923-131355_0_0_path.csv']
        #bad_files = ['20220921-142425_0_1_path.csv','20220921-142756_1_1_path.csv','20220921-151430_0_1_path.csv','20220921-151518_0_1_path.csv','20220923-114847_0_1_path.csv','20220923-115057_0_1_path.csv','20220928-105905_0_1_path.csv','20220928-105914_3_1_path.csv']
        #e = '20220928-110115_2_1_path.csv'
        #ce.replay(args,os.path.join("c:\\data\\label\\",e))
        """
        counter = 0
        for file in good_files:
            print(counter, file)
            ret = ce.replay(args,os.path.join("c:\\data\\label\\",file))
            counter +=1
            if ret < 0:
                    print("Replay cancelled by user!")
                    return
        time.sleep(10)
        counter = 0            
        for file in bad_files:
            print(counter, file)
            ret = ce.replay(args,os.path.join("c:\\data\\label\\",file))
            counter +=1
            if ret < 0:
                    print("Replay cancelled by user!")
                    return
        """
        
    finally:
        print(f"TOTAL RUN TIME: {time.time()-program_start_time}")

if __name__ == '__main__':
    main()
