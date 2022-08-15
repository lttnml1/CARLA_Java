#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    7/22/2022
# Purpose:          

import argparse
from email import header
import os
import math
import pandas as pd
from sklearn.model_selection import train_test_split


def main():

    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        '--path',
        help='directory to process',
        default=None,
        type=str)
    argparser.add_argument(
        '--num_samples',
        help='number of samples',
        default=None,
        type=int)
    argparser.add_argument(
        '--zero_one_percentage',
        help='percentage of 0s to 1s: default is .5',
        default=.5,
        type=float)
    argparser.add_argument(
        '--train_test_split',
        help='percentage of data to train (1-percentage used to test)',
        default=.67,
        type=float)
    args = argparser.parse_args()

    number_of_samples = args.num_samples
    num_zeros = math.ceil(args.num_samples*args.zero_one_percentage)
    num_ones = number_of_samples - num_zeros
    train_size = math.ceil(args.num_samples*args.train_test_split)
    test_size = number_of_samples - train_size

    print(f"You have chosen to take data from .csv files located in: {args.path}")
    print(f"You have chosen ({args.num_samples}) total samples with:")
    print(f"\t0s:\t\t{num_zeros}")
    print(f"\t1s:\t\t{num_ones}")
    print(f"\tTrain Size:\t{train_size}")
    print(f"\tTest Size:\t{test_size}")

    data_files = []
    for dirName, subdirList, fileList in os.walk(args.path):
            for fileName in fileList:
                if ".csv" in fileName: data_files.append(os.path.join(dirName,fileName))
    
    print(f"The following data files have been found: ")
    for f in data_files:
        print(f)
    
    list = []
    first_file = True
    headers = []
    for f in data_files:
        df = pd.read_csv(f, index_col=None, header=0)
        if(first_file): 
            headers = df.columns
            first_file = False
        list.append(df)
    df = pd.concat(list, axis=0, ignore_index=True)

    print(f"After processing the data, here are the stats:")
    print(f"\tTotal Samples:\t{df.shape[0]}")
    print(f"\tZeros:\t{df['label'].value_counts()[0]}")
    print(f"\tOnes:\t{df['label'].value_counts()[1]}")

    if(number_of_samples > df.shape[0]):
        print(f"**Not possible: You requested {number_of_samples} total samples and there are only {df.shape[0]}")
        return 0
    if(num_zeros > df['label'].value_counts()[0]):
        print(f"**Not possible: You requested {num_zeros} ZEROS and there are only {df['label'].value_counts()[0]}")
        return 0
    if(num_ones > df['label'].value_counts()[1]):
        print(f"**Not possible: You requested {num_ones} ONES and there are only {df['label'].value_counts()[1]}")
        return 0

    zero_samples = df[df['label']==0].sample(n=num_zeros,replace=False)
    one_samples = df[df['label']==1].sample(n=num_ones,replace=False)
    total_samples = pd.concat([zero_samples,one_samples],ignore_index=True)

    trainingSet, testSet = train_test_split(total_samples, test_size=(1-args.train_test_split))
    print(f"Train size:\t{trainingSet.shape[0]}, Zeros: {trainingSet['label'].value_counts()[0]}, Ones: {trainingSet['label'].value_counts()[1]}")
    print(f"Test size:\t{testSet.shape[0]}, Zeros: {testSet['label'].value_counts()[0]}, Ones: {testSet['label'].value_counts()[1]}")
    print(df.columns.values.tolist)
    trainingSet.to_csv(os.path.join(args.path,"train.csv"),index=False, header=headers)
    testSet.to_csv(os.path.join(args.path,"test.csv"),index=False, header=headers)

if __name__ == '__main__':
    main()