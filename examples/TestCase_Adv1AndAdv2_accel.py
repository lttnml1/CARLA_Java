#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    8/1/2022
# Purpose:          Supposed to be for TestCase_Adv2AndAdv2_accel

from __future__ import print_function

import argparse
import collections
import datetime
from distutils.spawn import spawn
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref
import csv
import time

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
# -- Adversary ---------------------------------------------------------------
# ==============================================================================

class Adversary():

    def __init__(self, point_array, speed_array, accel_array, dest_array, cost):
        self.point_array = point_array
        self.speed_array = speed_array
        self.accel_array = accel_array
        self.dest_array = dest_array
        self.cost = cost
        self.frame = 0


# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):

    def __init__(self, carla_world, grid, args):
        """Constructor method"""
        self.world = carla_world
        self._grid = grid
        self._args = args

        
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        
        self.ego = None
        self.Adversary = None
    
    @staticmethod
    def get_2D_distance(loc1, loc2):
        return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)
    
    @staticmethod
    def get_vehicle_speed(actor):
        """Returns vehicle speed in km/hr"""
        vel = actor.get_velocity()
        speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)#m/s * 1km/1000m * 3600s/1hr = km/hr (i.e., m/s * 3.6 = km/hr)
        return speed

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.ego,
            self.Adversary]
        for actor in actors:
            if actor is not None:
                actor.destroy()   
        

# ==============================================================================
# -- Grid ---------------------------------------------------------------
# ==============================================================================

class Grid(object):
    def __init__(self, world, top: float, bottom: float, left: float, right: float, draw_time: int = 0):
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
        self.world = world
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right
        self.box_width = abs(self.right - self.left)/20
        self.box_height = abs(self.top - self.bottom)/20
        if(draw_time > 0): Grid.draw_grid(self, draw_time) 

    def return_location_from_grid(self, i: int, j: int, draw_time: int = 0):
        center_point_y = self.left + self.box_width*(j) + self.box_width/2
        center_point_x = self.top - self.box_height*(i) - self.box_height/2
        location = carla.Location(x=center_point_x,y=center_point_y,z=1)
        if(draw_time > 0): self.draw_location_on_grid(location, draw_time)
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

    def draw_location_on_grid(self, location, draw_time = 10):
        self.world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)
            
    def draw_grid(self, draw_time = 10):
        #draw vertical lines
        y=self.left
        for i in range(0,21):
            self.world.debug.draw_line(carla.Location(x=self.top,y=y,z=1), carla.Location(x=self.bottom,y=y,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            y+=self.box_width
        #draw horizontal lines
        x=self.bottom
        for i in range(0,21):
            self.world.debug.draw_line(carla.Location(x=x,y=self.left,z=1), carla.Location(x=x,y=self.right,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            x+=self.box_height

# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================

def game_loop(args):
    """
    Main loop of the simulation.
    """
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        sim_world = client.get_world()
        tm = client.get_trafficmanager(8000)

        settings = sim_world.get_settings()
        tm.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        if(args.no_render): settings.no_rendering_mode = True
        sim_world.apply_settings(settings)


        if(args.debug): grid = Grid(client.get_world(),-65,-100,-10,30, draw_time = 60)
        else: grid = Grid(client.get_world(),-65,-100,-10,30)   
        world = World(client.get_world(), grid, args)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=20,y=207.4,z=50),carla.Rotation(roll=0, pitch=-40,yaw=0)))      
        
        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        Adversary1_blueprint=blueprints.filter("vehicle.audi.etron")[0]
        Adversary1_blueprint.set_attribute('role_name', 'Adversary1')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        spawn_points = world.map.get_spawn_points()
        ego_spawn_point = world.map.get_waypoint(carla.Location(x=50,y=207.4,z=1),project_to_road=True, lane_type=(carla.LaneType.Driving))  
        world.ego = world.world.try_spawn_actor(ego_blueprint,ego_spawn_point.transform)
        print(world.ego)
        world.ego.set_autopilot(True,8000)
        
        Adversary_spawn_point = ego_spawn_point.get_left_lane() 
        world.Adversary = world.world.try_spawn_actor(Adversary1_blueprint,Adversary_spawn_point.transform)
        print(world.Adversary)
        
        

        #This is necessary to ensure vehicle "stabilizes" after "falling"
        for i in range (0,30):
            world.world.tick()

        #ego_agent = BasicAgent(world.ego, target_speed = 20,  opt_dict={'ignore_traffic_lights':'True','base_vehicle_threshold':30.0})
        #ego_agent = BehaviorAgent(world.ego, behavior='aggressive')
        #ego_dest = carla.Location(x=155,y=207.4,z=5)
        #waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
        #ego_agent.set_destination(waypoint.transform.location)
        #sim_world.debug.draw_point(waypoint.transform.location,size=0.2,color=carla.Color(0,255,0),life_time=10)

        #adv_agent = BasicAgent(world.Adversary, target_speed = 5)
        
        
        while True:
            world.world.tick()
            #world.ego.apply_control(ego_agent.run_step())
            #world.Adversary.apply_control(adv_agent.run_step())

    finally:

        if world is not None:
            settings = world.world.get_settings()
            tm.set_synchronous_mode(False)
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            #settings.no_rendering_mode = False
            world.world.apply_settings(settings)
            


            world.destroy()


# ==============================================================================
# -- main() --------------------------------------------------------------
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
    argparser.add_argument(
        '--debug',
        action='store_true',
        help='Draws graph and points')
    argparser.add_argument(
        '--debug_nums',
        action='store_true',
        help='Draws points with numbers visited for categorical')

    args = argparser.parse_args()
    
    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()