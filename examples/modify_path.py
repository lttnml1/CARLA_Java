#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    7/22/2022
# Purpose:          

import argparse
import os
import csv



def modify_files(args):
    tempFile = "tmp"
    num_rows_to_change = 9
    for dirName, subdirList, fileList in os.walk(args.path):
            for fileName in fileList:
                if 'Normal' in fileName:
                    with open(os.path.join(dirName,fileName), mode='r') as inFile, open(os.path.join(dirName,tempFile), "w",newline='') as outFile:
                        csv_reader = csv.reader(inFile)
                        csv_writer = csv.writer(outFile)
                        line_num = 0
                        for row in csv_reader:
                            colValues = []
                            if line_num < num_rows_to_change:
                                col_num = 0
                                for col in row:
                                    if(col_num == 1):
                                        new_val = col
                                        if col == "61": new_val = "100"
                                        elif col == "81": new_val = "101"
                                        elif col == "102": new_val = "102"
                                        elif col == "103": new_val = "103"
                                        elif col == "84": new_val = "104"
                                        elif col == "65": new_val = "105"
                                        elif col == "46": new_val = "106"
                                        elif col == "67": new_val = "107"
                                        elif col == "88": new_val = "108"
                                        colValues.append(new_val)
                                    else: colValues.append(col)
                                    col_num += 1
                                csv_writer.writerow(colValues)
                            else:
                                csv_writer.writerow(row)
                            line_num += 1
                    os.rename(os.path.join(dirName,tempFile),os.path.join(dirName,"mod="+str(fileName)))

def remove_mod(args):
    for dirName, subdirList, fileList in os.walk(args.path):
        for fileName in fileList:
            if "mod=" in fileName:
                os.rename(os.path.join(dirName,fileName),os.path.join(dirName,str(fileName).split("=")[1]))


def main():

    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--path',
        help='directory to process',
        default=None,
        type=str)
    args = argparser.parse_args()

    modify_files(args)
    #remove_mod(args)

                        
                    

        
      
        



if __name__ == '__main__':
    main()