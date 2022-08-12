#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    8/10/2022
# Purpose:          

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================
import subprocess
import os
import argparse

def main():

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--path',
        help='directory to process',
        default=None,
        type=str)
    argparser.add_argument(
        '--scores',
        help='file path to write scores to',
        default=None,
        type=str)
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')


    args = argparser.parse_args()
    
    path = args.path
    
    try:
        for dirName, subdirList, fileList in os.walk(path):
            fileList.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
            for fileName in fileList:
                call_string2 = "C:/Users/m.litton_local/anaconda3/envs/carla_windows/python.exe c:/Users/m.litton_local/CARLA_Java/examples/Execute_scenario.py --port " + str(args.port) + " --file " + path + fileName + " --no_render"
                call_string3 = "C:/Users/m.litton_local/anaconda3/envs/carla_windows/python.exe c:/Users/m.litton_local/CARLA_Java/examples/Execute_scenario.py --port " + str(args.port) + " --file " + path + fileName
                #call_string = "/home/littonml1/anaconda3/envs/carla/bin/python -W ignore /home/littonml1/CARLA_Java/examples/graphPart_5_24_22.py --sync --loop --port " + str(args.port) + " --file /home/littonml1/python_proj/Adversary1/" + fileName + " --no_render"
                #get return value of subprocess call
                result=subprocess.Popen(call_string2, shell = True, stdout=subprocess.PIPE).communicate()[0].decode('ascii').strip()       
                #append it to a file
                if('INFO' in result):
                    result = result.split('INFO')[0].strip()
                '''
                if(not result): 
                    print("nothing")
                    break
                else: 
                    f = open(args.scores,"a")
                    f.write(write_string)
                    f.close()
                '''
                if(not result):
                    write_string = fileName.split("_")[2]+":8888.888888"+"\n"
                else:
                    write_string = fileName.split("_")[2]+":"+result+"\n"
                #print(write_string.strip())
                f = open(args.scores,"a")
                f.write(write_string)
                f.close()
          

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()