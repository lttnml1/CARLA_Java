#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    8/5/2022
# Purpose:          Main script for running/retrieving data from simulation for Cross-Entropy - supposed to be as general as possible

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref
import csv
import rtamt

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.simple_agent import SimpleAgent
from agents.navigation.behavior_agent import BehaviorAgent

from enum import Enum
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

# ==============================================================================
# -- Helper Functions ----------------------------------------------------------
# ==============================================================================

def get_2D_distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)

def get_vehicle_speed(actor):
    """Returns vehicle speed in km/hr"""
    vel = actor.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)#m/s * 1km/1000m * 3600s/1hr = km/hr (i.e., m/s * 3.6 = km/hr)
    return speed

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

# ==============================================================================
# -- Scenario ------------------------------------------------------------------
# ==============================================================================

class Scenario(object):

    def __init__(self, args):
        self.file = args.file
        self.score = 0
        self.point_array = []
        self.speed_array = []
        self.accel_array = []
        self.destination_array = []

        self.read_file()

    def read_file(self):
        with open(self.file, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                self.point_array.append(row[1])
                self.speed_array.append(float(row[2]) * 3.6)
                self.accel_array.append(float(row[3]))
    
    def score_path(self):
        isBadPath = False
        for i in range(0,len(self.point_array)):
            speed = self.speed_array[i]
            #no NaN speed
            if math.isnan(speed):
                isBadPath = True
                self.score = sys.float_info.max/2000
                break
            #speed is non-negative    
            elif speed<0:
                self.score += 100*abs(speed)
                break
        return isBadPath


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):

    def __init__(self, carla_world, args, grid):
        self.world = carla_world
        self._args = args
        self._grid = grid

        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        
        self.ego = None
        self.adversary = None
    
    def destroy(self):
        actors = [
            self.adversary,
            self.ego]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def draw_location_on_grid(self, location, draw_time = 10):
        self.world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)
            
    def draw_grid(self, draw_time = 10):
        #draw vertical lines
        y=self._grid.left
        for i in range(0,21):
            self.world.debug.draw_line(carla.Location(x=self._grid.top,y=y,z=1), carla.Location(x=self._grid.bottom,y=y,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            y+=self._grid.box_width
        #draw horizontal lines
        x=self._grid.bottom
        for i in range(0,21):
            self.world.debug.draw_line(carla.Location(x=x,y=self._grid.left,z=1), carla.Location(x=x,y=self._grid.right,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            x+=self._grid.box_height

    def convert_points_to_locations(self, scenario: Scenario):
        for point in scenario.point_array:
            i, j = self._grid.return_coords_from_point(point) 
            dest = self._grid.return_location_from_grid(i,j)
            scenario.destination_array.append(dest)


# ==============================================================================
# -- Grid ---------------------------------------------------------------
# ==============================================================================

class Grid(object):
    def __init__(self, top: float, bottom: float, left: float, right: float, draw_time: int = 0):
        #this will be a 20x20 grid
        """
        top/bottom are x values
        left/right are y values
        assume axes look like
        x
        ^
        |
        |
        |
        --------> Y
        """
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right
        self.box_width = abs(self.right - self.left)/20
        self.box_height = abs(self.top - self.bottom)/20 

    def return_location_from_grid(self, i: int, j: int, draw_time: int = 0):
        center_point_y = self.left + self.box_width*(j) + self.box_width/2
        center_point_x = self.top - self.box_height*(i) - self.box_height/2
        location = carla.Location(x=center_point_x,y=center_point_y,z=1)
        return location
    
    def return_grid_from_location(self, location):
        for index in range(0,19):
            if(location.x < self.top - self.box_height * (index)): i=index
            if(location.y > self.left + self.box_width* (index)): j=index
        return (i,j)
    
    def return_coords_from_point(self, point):
        i = math.floor(int(point)/20)
        j = int(point) % 20
        return (i,j)
    
    def return_point_from_coords(self, i, j):
        return i * 20 + j

# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================

def game_loop(args):

    world = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        sim_world = client.get_world()

        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        if(args.no_render): settings.no_rendering_mode = True
        sim_world.apply_settings(settings)

        grid = Grid(-65,-100,-10,30)
        scenario = Scenario(args)
        world = World(client.get_world(), args, grid)
        world.convert_points_to_locations(scenario)

        #ensure path is good - like AbstractScore.java
        if(scenario.score_path()):
            return 0

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))

        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        adversary_blueprint=blueprints.filter("vehicle.diamondback.century")[0]
        adversary_blueprint.set_attribute('role_name', 'adversary')
        adversary_spawn_point = carla.Transform(scenario.destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        world.adversary = world.world.try_spawn_actor(adversary_blueprint,adversary_spawn_point)

        #This is necessary to ensure vehicle "stabilizes" after "falling"
        for i in range (0,30):
            world.world.tick()

        execute_scenario(world, scenario)

        score_scenario(world, scenario)        

    finally:
        print(f"{scenario.score:8.6f}")

        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            #settings.no_rendering_mode = False
            world.world.apply_settings(settings)

            world.destroy()

# ==============================================================================
# -- execute_scenario() --------------------------------------------------------
# ==============================================================================

def execute_scenario(world, scenario):
    dest_index = 1
    #set initial destination and target_speed
    destination = scenario.destination_array[dest_index]
    adversary_loca = world.adversary.get_location()
    distance = get_2D_distance(adversary_loca,destination)
    target_speed = 0 + scenario.accel_array[dest_index-1] * (distance/1 * 3.6)  
    adversary_agent = SimpleAgent(world.adversary, destination, target_speed=target_speed)

    big_array = []
    
    while True:
        world.world.tick()
        adversary_loca = world.adversary.get_location()
        adversary_speed = get_vehicle_speed(world.adversary)

        


        if adversary_agent.done():
            small_array = []
            small_array.append(dest_index-1)
            small_array.append(adversary_speed)
            big_array.append(small_array)
               
            if (dest_index >= len(scenario.destination_array)-1):
                    #print("Adversary's route is complete, breaking out of loop")
                    break
            else:
                dest_index += 1
                new_dest = scenario.destination_array[dest_index]
                current_speed = adversary_speed
                distance = get_2D_distance(adversary_loca,new_dest)
                target_speed = current_speed + scenario.accel_array[dest_index-1] * (distance/current_speed * 3.6)  
                #print(f"target speed ({target_speed:4.2f}) = current speed ({current_speed:4.2f} km/hr) + accel ({adversary.accel_array[dest_index-1]:4.2f} km/(hr*s)) * (distance ({distance:4.2f} meters) / speed ({current_speed:4.2f} km\hr) * 3.6 km*sec/(m*hr))")

                adversary_agent.set_destination(new_dest)
                adversary_agent.set_target_speed(target_speed)

        control = adversary_agent.run_step()
        control.manual_gear_shift = False
        world.adversary.apply_control(control)
    
    file = "c:\\data\\test_csv.csv"
    header = ['time','speed']
    with open(file,'w',newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(header)
        csvwriter.writerows(big_array)

        

# ==============================================================================
# -- score_scenario() ----------------------------------------------------------
# ==============================================================================

def score_scenario(world, scenario):
    
    dataSet1 = {
        'time': [0,1,2,3,4],
        'speed': [8.678609289, 9.555141637, 8.995255402, 9.656090744, 10.22768276]
    }
    dataSet = {
         'time': [0, 1, 2],
         'a': [100.0, -1.0, -2.0],
         'b': [20.0, 2.0, -10.0]
    }
    spec = rtamt.STLDiscreteTimeSpecification()
    spec.name = 'STL discrete-time online Python monitor'
    spec.declare_var('speed', 'float')
    spec.spec = 'speed>10'

    try:
        spec.parse()
        spec.pastify()
    except rtamt.STLParseException as err:
        print('STL Parse Exception: {}'.format(err))
        sys.exit()

    rob = spec.evaluate(dataSet1)
    print('Robustness: ' + str(rob))

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--no_render',
        action = 'store_true',
        help='Render graphics (default: False)')
    argparser.add_argument(
        '--file',
        help='file name to read from',
        default=None,
        type=str)

    args = argparser.parse_args()
    
    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()