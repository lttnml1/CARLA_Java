#!/usr/bin/env python
import sys
import csv
import rtamt
import os

from rtamt.spec.stl.discrete_time.specification import Semantics

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


def monitor():

    read_file = 'c:\\data\\log_file.csv'
    write_file = 'c:\\data\\rob.csv'
    dataSet = read_csv(read_file)
    
    spec = rtamt.STLDiscreteTimeSpecification()
    spec.name = 'Test'
    spec.declare_var('distance', 'float')
    spec.declare_var('ego_speed', 'float')
    spec.declare_var('out', 'float')
    spec.set_var_io_type('distance', 'input')
    spec.set_var_io_type('ego_speed', 'output')
    spec.spec = 'out = ((distance < 5.0)  implies (eventually[0:10](ego_speed < 1)))'
    spec.semantics = Semantics.STANDARD

    try:
        spec.parse()
        spec.pastify()
    except rtamt.STLParseException as err:
        print('STL Parse Exception: {}'.format(err))
        sys.exit()

    rob_vals = []
    for i in range(len(dataSet['distance'])):
        rob = spec.update(i, [('distance', dataSet['distance'][i]), ('ego_speed', dataSet['ego_speed'][i])])
        rob_vals.append((dataSet['time'][i],rob))
    
    with open(write_file,'w',newline='') as writer:
        csv_writer = csv.writer(writer)
        csv_writer.writerows(rob_vals)





if __name__ == '__main__':
    # Process arguments
    
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    monitor()