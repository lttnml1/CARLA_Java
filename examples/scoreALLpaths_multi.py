#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    5/24/2022
# Purpose:          Ego goes from right to left, Adv1 goes from bottom to top at normally distributed speed

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================
from re import sub
import subprocess
import os
import argparse
import shutil

def move_files(args):
    files = [os.path.join(args.path, f) for f in os.listdir(args.path)]
    N=len(files) / args.num_servers
    i=0
    curr_subdir = None
    files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
    for f in files:
        if i%N == 0:
                subdir_name = os.path.join(args.path, f'{int(i // N + 1)}')
                os.mkdir(subdir_name)
                curr_subdir = subdir_name
        f_base = os.path.basename(f)
        shutil.move(f,os.path.join(subdir_name,f_base))
        i +=1

def main():

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--path',
        help='directory to process',
        default="c:\\data\\Test\\",
        type=str)
    argparser.add_argument(
        '--scores',
        help='file path to write scores to',
        default="c:\\data\\ScoresTest.txt",
        type=str)
    argparser.add_argument(
        '-n', '--num_servers',
        metavar='N',
        default=2,
        type=int,
        help='Number of servers to run (default: 2)')


    args = argparser.parse_args()
    
    path = args.path
    
    try:
        #divide files in path between n servers
        move_files(args)

        dirs_list = []
        for dir in os.listdir(args.path):
            d = os.path.join(args.path,dir)
            if os.path.isdir(d):
                dirs_list.append(d)

        commands = []
        port = 2000
        i = 1
        for dir in dirs_list:
            command = "C:/Users/m.litton_local/anaconda3/envs/carla_windows/python.exe c:/Users/m.litton_local/CARLA_Java/examples/scoreALLPaths.py --path " + dir + "\ --scores " + "c:\\data\\Adversary1\\ScoresTest" + os.path.basename(dir) + ".txt" + " --port " + str(port + 4*i)
            commands.append(command) 
            i+=1
        
        for cmd in commands:
            print(cmd)

        processes = [subprocess.Popen(cmd,shell = False, stdout=subprocess.PIPE,stderr=subprocess.PIPE) for cmd in commands]

        for p in processes:
            stdout, stderr = p.communicate()
            print(stdout.decode('ascii'))    
        
        print("Done - combining files")

        scores_list = []
        for dir in os.listdir(args.path):
            d = os.path.join(args.path,dir)
            if os.path.isfile(d):
                scores_list.append(d)
        
        with open(args.scores,"w") as combined_file:
            for filename in scores_list:
                with open(filename) as infile:
                    contents = infile.read()
                    combined_file.write(contents)
        
        print(f"Files are now combined - see output file at {args.scores}")


    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()