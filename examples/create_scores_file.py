#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    7/22/2022
# Purpose:          

import argparse



def main():

    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        '--t_value',
        help='t value',
        default=None,
        type=str)
    argparser.add_argument(
        '--start',
        help='start',
        default=None,
        type=str)
    argparser.add_argument(
        '--end',
        help='end',
        default=None,
        type=str)
    args = argparser.parse_args()

    file = f"C:\\data\\Scores\\TESTt{args.t_value}_Adversary3_Scores.txt"
    score = 0.0
    with open(file,mode='w') as f:
        for i in range(int(args.start),int(args.end)+1):
            f.write(f"NormalDistrib#{i}:{score:8.6f}\n")

        
      
        



if __name__ == '__main__':
    main()