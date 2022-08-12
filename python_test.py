#!/usr/bin/env python

import random

start_points_dict = {'start_points':[15,228,61,98]}
end_points_dict = {}
end_points_dict.update({15:[229,58,9,90,91]})
end_points_dict.update({228:[58,9,90,91,31]})
end_points_dict.update({61:[229,31,90,91,9]})
end_points_dict.update({98:[58,229,31,90,91]})
    
start_point = random.choice(start_points_dict['start_points'])
end_point = random.choice(end_points_dict[start_point])
print(f"start:{start_point}, end:{end_point}")
