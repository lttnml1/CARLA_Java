#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    7/22/2022
# Purpose:          

import argparse
import os
import csv

def record_five_minutes(index, rows, label):
    data = []
    line_count = 0
    for row in rows:
        if((line_count-index)%10==0 and line_count<=index + 100 and line_count>=index): 
            data.append(row[1:])
        line_count += 1
    line = []
    time_index = 0
    for d in data:
        line.append(int(time_index))
        for item in d:
            line.append(float(item))
        time_index += 1
    line.append(int(label))
    return line
    
def write_to_file(line, file):
    with open(file,'a',newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(line)

def main():

    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        '--path',
        help='directory to process',
        default=None,
        type=str)
    args = argparser.parse_args()

    illegal_files = []
    for dirName, subdirList, fileList in os.walk(args.path):
        for fileName in fileList:
            file_feature_vector = []
            filename_components = str(fileName).split("_",4)
            label = filename_components[2].split("label")[1]
            frame = filename_components[4].split("frame")[1].split(".")[0]
            #print(f"File {str(fileName)} has label of {label} and event of interest is at frame {frame}")
            row_of_interest = 0
            index = 0
            fields = []
            rows = []
            with open(os.path.join(dirName,fileName), mode='r') as csv_file:
                csv_reader = csv.reader(csv_file)
                fields = next(csv_reader)
                line_count = 0
                for row in csv_reader:
                    rows.append(row)
                    if(row[0]==frame):
                        row_of_interest = line_count
                    line_count+=1
                if(row_of_interest >= 100): index = row_of_interest-100
                elif((row_of_interest < 100 and row_of_interest > 0) or frame == "100"): index = 0
                else: 
                    index = 'illegal'
                    illegal_files.append((fileName, row_of_interest))
                if(index != 'illegal'):
                    line = record_five_minutes(index,rows,label)
                    write_to_file(line, os.path.join(os.path.dirname(args.path),"features_list.csv"))
    print(f"Done writing, there are {len(illegal_files)} illegal files")
    if(len(illegal_files) > 0):
        for file in illegal_files:
            print(file)        


if __name__ == '__main__':
    main()