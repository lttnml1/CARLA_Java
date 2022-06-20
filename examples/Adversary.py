#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    6/20/2022
# Purpose:          

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

from enum import Enum


# ==============================================================================
# -- Adversary ---------------------------------------------------------------
# ==============================================================================
class Distribution_Type(Enum):
    NORMAL = 1
    CATEGORICAL = 2

class Adversary():
    def __init__(self, distribution_type: Distribution_Type):
        self.distribution_type = distribution_type
        self.point_array = []
        self.speed_array = []
        self.accel_array = []
        self.destination_array = []
    def read_data_from_file(file):
        pass
    def compute_score():
        pass

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
        self.Adversary_1 = None
        self.collision_sensor = None
    
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
            self.Adversary_1,
            self.collision_sensor.sensor]
        for actor in actors:
            if actor is not None:
                actor.destroy()



# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================
class CollisionSensor(object):
    """ Class for collision sensors"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        """Gets the history of collisions"""
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        """On collision method"""
        self = weak_self()
        if not self:
            return
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
        #if "vehicle" in str(event.other_actor.type_id):
        #    print(f"Collision with {event.other_actor.type_id}")
        global COLLISION
        COLLISION = True

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

        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        if(args.no_render): settings.no_rendering_mode = True
        sim_world.apply_settings(settings)

        grid = Grid(client.get_world(),-33,-61,4,38, draw_time = 30)   
        world = World(client.get_world(), grid, args)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-47,y=21,z=30),carla.Rotation(roll=0, pitch=-90,yaw=0)))
        
        #figure out what kind of distribution type it is
        adversary = None
        if "Normal" in str(args.file):
            adversary = Adversary(Distribution_Type.NORMAL)
        if "Categorical" in str(args.file):
            adversary = Adversary(Distribution_Type.CATEGORICAL)
        else:
            print("This file type is unrecognized!!")

        #read the path the Adversary is supposed to take from the file
        with open(args.file) as f:
            lines = f.readlines()
        for line in lines:
            sp = line.split(',')
            adversary.point_array.append(sp[1])
            adversary.speed_array.append(float(sp[2]) * 3.6)
            adversary.accel_array.append(float(sp[3]) * 3.6) #converts accel in m/(s * s) to km/(hr * s)
            
        for point in adversary.point_array:
            i, j = grid.return_coords_from_point(point) 
            dest = grid.return_location_from_grid(i,j)
            adversary.destination_array.append(dest)
        
        print(f"The size of destination_array is {len(destination_array)}")      
        
        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        Adversary1_blueprint=blueprints.filter("diamondback")[0]
        Adversary1_blueprint.set_attribute('role_name', 'Adversary1')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')


        Adversary1_spawn_point = carla.Transform(destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        world.Adversary_1 = world.world.try_spawn_actor(Adversary1_blueprint,Adversary1_spawn_point)
        grid.draw_location_on_grid(destination_array[0], draw_time = 5)
        print(f"Adversary 1 has been spawned at {grid.return_grid_from_location(destination_array[0])}")
        
        ego_spawn_point = carla.Transform(grid.return_location_from_grid(8,20),carla.Rotation(roll=0,pitch=0,yaw=-90))
        world.ego = world.world.try_spawn_actor(ego_blueprint,ego_spawn_point)

        world.collision_sensor = CollisionSensor(world.Adversary_1)    
        
        #This is necessary to ensure vehicle "stabilizes" after "falling"
        for i in range (0,30):
            world.world.tick()
        
        #now intialize the agents
        dest_index = 1
        Adversary1_loca = world.Adversary_1.get_location()
        i,j = grid.return_grid_from_location(Adversary1_loca)
        point = grid.return_point_from_coords(i,j)
        print(f"Adversary1 is located at ({Adversary1_loca.x},{Adversary1_loca.y}), grid: ({i},{j}) -> {point}")
        
        current_speed = world.get_vehicle_speed(world.Adversary_1)
        print(f"Adversary1's speed is {current_speed:4.2f}")

        destination = destination_array[dest_index]
        i,j = grid.return_grid_from_location (destination)
        point = grid.return_point_from_coords(i,j)
        print(f"Adversary1 is headed to ({destination.x},{destination.y}), grid: ({i},{j}) -> {point}")

        distance = world.get_2D_distance(Adversary1_loca,destination)
        print(f"The distance between those two points is {distance:4.2f} meters")
        
        
        target_speed = 0 + accel_array[dest_index-1] * (distance/10.0 * 3.6)  
        print(f"target speed ({target_speed:4.2f}) = current speed ({current_speed:4.2f} km/hr) + accel ({accel_array[dest_index-1]:4.2f} km/(hr*s)) * (distance ({distance:4.2f} meters) / speed ({current_speed:4.2f} km\hr) * 3.6 km*sec/(m*hr))")
        
        Adversary1_agent = SimpleAgent(world.Adversary_1, destination, target_speed=target_speed)
        grid.draw_location_on_grid(destination_array[1], draw_time = 5)
                
        ego_agent = BasicAgent(world.ego, target_speed = 7)
        ego_dest = carla.Location(x=-40,y=0,z=0)
        waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
        ego_agent.set_destination(waypoint.transform.location)
        
        
        while True:
            world.world.tick()

            Adversary1_speed = world.get_vehicle_speed(world.Adversary_1)
            ego_speed = world.get_vehicle_speed(world.ego)    

            ego_loca = world.ego.get_location()
            Adversary1_loca = world.Adversary_1.get_location()
            
            if Adversary1_agent.done():
                print(world.world.get_snapshot().timestamp)
                i,j = grid.return_grid_from_location(destination_array[dest_index])
                point = grid.return_point_from_coords(i,j)
                print(f"Adversary 1 has reached destination {dest_index} at point ({point}):\tSpeed is actually {Adversary1_speed} km/hr")
                                          
                if (dest_index >= len(destination_array)-1):
                    print("Adversary 1's route is complete, breaking out of loop")
                    break
                else:
                    print("Setting new destination for Adversary1") 
                    dest_index = dest_index+1
                    print(f"Dest_index: {dest_index}")
                    new_dest = destination_array[dest_index]
                    grid.draw_location_on_grid(new_dest, draw_time = 3)
                    
                    current_speed = Adversary1_speed
                    print(f"Adversary1's speed is {current_speed:4.2f}")

                    i,j = grid.return_grid_from_location(new_dest)
                    point = grid.return_point_from_coords(i,j)
                    print(f"Adversary1 is headed to ({new_dest.x},{new_dest.y}), grid: ({i},{j}) -> {point}")

                    distance = world.get_2D_distance(Adversary1_loca,new_dest)
                    print(f"The distance between those two points is {distance:4.2f} meters")
                    
                    #if(current_speed == 0): current_speed = 10
                    target_speed = current_speed + accel_array[dest_index-1] * (distance/current_speed * 3.6)  
                    print(f"target speed ({target_speed:4.2f}) = current speed ({current_speed:4.2f} km/hr) + accel ({accel_array[dest_index-1]:4.2f} km/(hr*s)) * (distance ({distance:4.2f} meters) / speed ({current_speed:4.2f} km\hr) * 3.6 km*sec/(m*hr))")

                    Adversary1_agent.set_destination(new_dest)
                    Adversary1_agent.set_target_speed(target_speed)
                                        
            
            control = Adversary1_agent.run_step()
            control.manual_gear_shift = False
            world.Adversary_1.apply_control(control)
            if (ego_loca.y > grid.left):
                world.ego.apply_control(ego_agent.run_step())   
            else:
                world.ego.apply_control(ego_agent.add_emergency_stop(carla.VehicleControl()))        

    finally:
        if world is not None:
            settings = world.world.get_settings()
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

    args = argparser.parse_args()
    
    try:
        global COLLISION
        COLLISION = False
        global MIN_DISTANCE
        MIN_DISTANCE = 999.99999
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()