#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    5/24/2022
# Purpose:          Ego goes from right to left, Adv1 goes from bottom to top at normally distributed speed

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================
import subprocess
import os

def main():
    path = "/home/littonml1/python_proj/Adversary1/"
    
    try:
        for dirName, subdirList, fileList in os.walk(path):
            fileList.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
            for fileName in fileList:
                call_string = "/home/littonml1/anaconda3/envs/carla/bin/python -W ignore /home/littonml1/CARLA_Java/examples/graphPart_5_24_22.py --sync --loop --file /home/littonml1/python_proj/Adversary1/" + fileName + " --no_render"
                #get return value of subprocess call
                result=subprocess.Popen(call_string, shell = True, stdout=subprocess.PIPE).communicate()[0].decode('ascii').strip()       
                #append it to a file
                write_string = fileName.split("_")[1]+":"+result+"\n"
                f = open("/home/littonml1/python_proj/000_Adversary1_Scores", "a")
                f.write(write_string)
                f.close()  

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()