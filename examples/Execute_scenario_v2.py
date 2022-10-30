#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    10/28/2022
# Purpose:          Execute scenarios from Java files

from __future__ import print_function

import argparse
import glob
import math
import os
import sys
import csv

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

# ==============================================================================
# -- Helper Functions ----------------------------------------------------------
# ==============================================================================

def get_2D_distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)

def get_vehicle_speed(actor):
    """Returns vehicle speed in km/hr"""
    vel = actor.get_velocity()
    speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)#m/s * 1km/1000m * 3600s/1hr = km/hr (i.e., m/s * 3.6 = km/hr, 3.6 = (km*s)/(m*hr))
    return speed

# ==============================================================================
# -- Scenario ------------------------------------------------------------------
# ==============================================================================

class Scenario(object):

    def __init__(self, args):
        self._args = args
        self.file = args.file
        self.point_array = []
        self.speed_array = []
        self.destination_array = []
        self.read_file()

    def read_file(self):
        with open(self.file, mode='r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                self.point_array.append(row[1])
                self.speed_array.append(float(row[2]) * 3.6 * 3)

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

    def draw_location_on_grid(self, location, str_to_draw, draw_time = 10):
        #self.world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)
        self.world.debug.draw_string(location,str_to_draw,draw_shadow=False,color=carla.Color(0,255,0),life_time=draw_time)


    def draw_grid(self, draw_time = 10):
        #draw vertical lines
        y=self._grid.left
        for i in range(0,21):
            self.world.debug.draw_line(carla.Location(x=self._grid.top,y=y,z=self._grid.grid_height), carla.Location(x=self._grid.bottom,y=y,z=self._grid.grid_height), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            y+=self._grid.box_width
        #draw horizontal lines
        x=self._grid.bottom
        for i in range(0,21):
            self.world.debug.draw_line(carla.Location(x=x,y=self._grid.left,z=self._grid.grid_height), carla.Location(x=x,y=self._grid.right,z=self._grid.grid_height), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            x+=self._grid.box_height

    def convert_points_to_locations(self, scenario: Scenario):
        for point in scenario.point_array:
            i, j = self._grid.return_coords_from_point(point) 
            dest = self._grid.return_location_from_grid(i,j)
            scenario.destination_array.append(dest)
    
    def draw_points_and_locations(self, points):
        counter = 0
        for point in points:
            self.world.debug.draw_point(point.location,size=0.2,color=carla.Color(255,255,255),life_time=30)
            #self.world.debug.draw_string(point.location + carla.Location(z=3),str(point.location.x)+',\n'+str(point.location.y),color=carla.Color(255,0,0),life_time=30)
            self.world.debug.draw_string(point.location + carla.Location(z=3),str(counter),color=carla.Color(255,0,0),life_time=30)
            counter += 1
    
    def draw_spawn_point_locations(self):
        spawn_points = self.map.get_spawn_points()
        for i in range(len(spawn_points)):
            self.world.debug.draw_string(spawn_points[i].location,f"{i}",life_time = 30)

# ==============================================================================
# -- Grid ---------------------------------------------------------------
# ==============================================================================

class Grid(object):
    def __init__(self, top: float, bottom: float, left: float, right: float, grid_height = 1, draw_time: int = 0):
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

        self.grid_height = grid_height

    def return_location_from_grid(self, i: int, j: int):
        center_point_y = self.left + self.box_width*(j) + self.box_width/2
        center_point_x = self.top - self.box_height*(i) - self.box_height/2
        location = carla.Location(x=center_point_x,y=center_point_y,z=self.grid_height)
        return location
    
    def return_grid_from_location(self, location):
        i = 0
        j = 0
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

def game_loop(args, scenario):

    world = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        
        sim_world = client.get_world()
        
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        sim_world.apply_settings(settings)


        grid = Grid(30,-100,24,40,11, draw_time=10)
        world = World(client.get_world(), args, grid)
        world.convert_points_to_locations(scenario)
        world.draw_grid()

        spectator = world.world.get_spectator()

        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        adversary_blueprint=blueprints.filter("vehicle.jeep.wrangler_rubicon")[0]
        adversary_blueprint.set_attribute('role_name', 'adversary')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        #adversary_spawn_point = carla.Transform(scenario.destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        wp = world.map.get_waypoint(scenario.destination_array[0],project_to_road=False,lane_type=carla.LaneType.Any)
        adversary_spawn_point = wp.transform
        adversary_spawn_point.location.z += 2
        adversary_spawn_point.rotation.yaw += 180
        world.adversary = world.world.try_spawn_actor(adversary_blueprint,adversary_spawn_point)

        
        #world.ego = world.world.try_spawn_actor(ego_blueprint, ego_spawn_point)

        #This is necessary to ensure vehicle "stabilizes" after "falling"
        for i in range (0,30):
            world.world.tick()
        
        for i in range(len(scenario.destination_array)):
            if(i==0):
                world.draw_location_on_grid(scenario.destination_array[i],'START',draw_time=7)
            elif(i==len(scenario.destination_array)-1):
                world.draw_location_on_grid(scenario.destination_array[i],'END',draw_time=7)
            else: world.draw_location_on_grid(scenario.destination_array[i],str(i),draw_time=7)

        execute_scenario(world, scenario, spectator)    

    finally:
        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)

            world.destroy()

# ==============================================================================
# -- execute_scenario() --------------------------------------------------------
# ==============================================================================

def execute_scenario(world, scenario, spectator):

    dest_index = 1

    #set initial destination and target_speed
    destination = scenario.destination_array[dest_index]
    target_speed = scenario.speed_array[dest_index]
    
    adversary_agent = SimpleAgent(world.adversary, destination, target_speed=target_speed)
    #ego_agent = BasicAgent(world.ego, target_speed = 9,  opt_dict={'ignore_traffic_lights':'True','base_vehicle_threshold':10.0})
    #ego_agent.set_destination(world.map.get_spawn_points()[scenario.ego_end].location)


    while True:
        world.world.tick()
        
        if adversary_agent.done():
               
            if (dest_index >= len(scenario.destination_array)-1):
                    print("Adversary's route is complete, breaking out of loop")
                    break
            else:
                dest_index += 1
                new_dest = scenario.destination_array[dest_index]
                target_speed = scenario.speed_array[dest_index] 
                
                adversary_agent.set_destination(new_dest)
                adversary_agent.set_target_speed(target_speed)


        control = adversary_agent.run_step()
        control.manual_gear_shift = False
        world.adversary.apply_control(control)

        #world.ego.apply_control(ego_agent.run_step())

        
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
        default=2004,
        type=int,
        help='TCP port to listen to (default: 2004)')
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
        #first read the path from the file
        scenario = Scenario(args)

        
        game_loop(args, scenario)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()