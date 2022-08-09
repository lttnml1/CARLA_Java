#!/usr/bin/env python
import sys
import csv
import rtamt
import os

def read_csv(filename):
    f = open(filename, 'r')
    reader = csv.reader(f)
    headers = next(reader, None)

    column = {}
    for h in headers:
        column[h] = []

    for row in reader:
        for h, v in zip(headers, row):
            column[h].append(float(v))

    return column


import sys
import rtamt

def monitor():
    # data
    dataSet = {
         'time': [0, 1, 2],
         'a': [100.0, -1.0, -2.0],
         'b': [20.0, 2.0, -10.0]
    }

    dataSet1 = read_csv('c:\\Users\\m.litton_local\\CARLA_Java\\STL_Monitor\\example1.csv')

    # # stl
    spec = rtamt.STLDiscreteTimeSpecification()
    spec.name = 'STL discrete-time online Python monitor'
    spec.declare_var('req', 'float')
    spec.declare_var('gnt', 'float')
    spec.spec = 'always((req>=3) implies (eventually[0:5](gnt>=3)))'

    try:
        spec.parse()
    except rtamt.STLParseException as err:
        print('STL Parse Exception: {}'.format(err))
        sys.exit()

    rob = spec.evaluate(dataSet1)
    print(str(rob))
    if(len(rob)>0):
        min_rob = rob[0][1]
        for r in rob:
            if r[1] < min_rob:
                min_rob = rob[1]
        print(f"Minimum robustness: {str(min_rob)}")

if __name__ == '__main__':
    # Process arguments

    monitor()