#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    8/15/2022
# Purpose:          

import argparse
from email import header
import os
import math
import pandas as pd
from collections import Counter


def main():

    argparser = argparse.ArgumentParser()

    argparser.add_argument(
        '--file',
        help='file to process',
        default=None,
        type=str)
    args = argparser.parse_args()

    print(f"You are modifying file: {args.file}")
    
    df = pd.read_csv(args.file, index_col=None, header=0)
    headers = df.columns

    #figure out which columns you want to calculate from
    position_columns = []
    for h in headers:
        if "_loc_" in h:
            position_columns.append(h)
    print(f"Position columns are: {position_columns}")
    

    #insert columns for distance
    distance_columns = []
    for pc in position_columns:
        if "ego_loc_y" in pc:
            insert_loca = df.columns.get_loc(pc)
            number = pc.split("_",3)[3]
            column_name = "distance_"+number
            distance_columns.append(column_name)
            df.insert(insert_loca+1,column_name,0.000000)

    #update distance columns
    for index_label, row_series in df.iterrows():
        for dc in distance_columns:
            num = dc.split("_")[1]
            dist = math.sqrt((row_series[str('ego_loc_x_'+num)]-row_series[str('adv_loc_x_'+num)])**2 + (row_series[str('ego_loc_y_'+num)]-row_series[str('adv_loc_y_'+num)])**2)
            df.at[index_label , str('distance_'+num)] = dist


    #drop the location columns
    df.drop(position_columns,axis=1, inplace=True)
    df.to_csv(args.file.split(".")[0]+"_mod.csv",index=False)

    


if __name__ == '__main__':
    main()