#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    8/8/2022
# Purpose:          Adversary3 has normally distributed acceleration and categorically distributed position, **TOWN 03**


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
import time
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

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



# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    """Method to get actor display name"""
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name



# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """

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
        self.Adversary_3 = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        
        
        
        self.recording_enabled = False
        self.recording_start = 0

    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.ego.get_world().set_weather(preset[0])
    
    def get_crosswalk(self, draw_time: int = 0):
        crosswalks = self.map.get_crosswalks()
        crosswalk_array = []

        single_crosswalk = []
        for i in range(0,len(crosswalks)):
            single_crosswalk.append(crosswalks[i])
            if(len(single_crosswalk) > 1):
                if(single_crosswalk[0] == single_crosswalk[-1]):
                    crosswalk_array.append(single_crosswalk)
                    single_crosswalk = []
        
        #of_interest = [crosswalk_array[2],crosswalk_array[44],crosswalk_array[45],crosswalk_array[46], crosswalk_array[47]]
        if(draw_time > 0):
            for cw in crosswalk_array:    
                for j in range (0,len(cw)-1):
                    self.world.debug.draw_line(cw[j]+carla.Location(z=1), cw[j+1]+carla.Location(z=1), thickness=0.2, color=carla.Color(255,0,0), life_time=draw_time)
        
        return crosswalk_array
        # Location(x=-97.195564, y=23.552834, z=0.038310)
        # Location(x=-94.079308, y=24.415840, z=0.031020)
        # Location(x=-67.668266, y=5.793464, z=-0.064348)
        # Location(x=-70.731575, y=4.893159, z=-0.058300)       
        
    
    def is_in_crosswalk(self, actor_location):
        point = Point(actor_location.x,actor_location.y)

        outer_polygon = []
        outer_polygon.append(carla.Location(x=-97,y=26,z=1))
        outer_polygon.append(carla.Location(x=-68.5,y=5.5,z=1))
        outer_polygon.append(carla.Location(x=-68.5,y=-7.5,z=1))
        outer_polygon.append(carla.Location(x=-96,y=-7.5,z=1))
        outer_polygon.append(carla.Location(x=-97,y=26,z=1))

        #for j in range (0,len(outer_polygon)-1):
        #    self.world.debug.draw_line(outer_polygon[j]+carla.Location(z=1), outer_polygon[j+1]+carla.Location(z=1), thickness=0.2, color=carla.Color(255,0,0), life_time=20)
        
        inner_polygon = []
        inner_polygon.append(carla.Location(x=-94,y=20,z=1))
        inner_polygon.append(carla.Location(x=-71.6,y=3.6,z=1))
        inner_polygon.append(carla.Location(x=-71.6,y=-4.5,z=1))
        inner_polygon.append(carla.Location(x=-93.3,y=-4.5,z=1))
        inner_polygon.append(carla.Location(x=-94,y=20,z=1))

        #for j in range (0,len(inner_polygon)-1):
        #    self.world.debug.draw_line(inner_polygon[j]+carla.Location(z=1), inner_polygon[j+1]+carla.Location(z=1), thickness=0.2, color=carla.Color(0,255,0), life_time=20)
        inner_polygon_points = []
        for pt in inner_polygon:
            inner_polygon_points.append((pt.x,pt.y))
        outer_polygon_points = []
        for pt in outer_polygon:
            outer_polygon_points.append((pt.x,pt.y))
        polygon_outer = Polygon(outer_polygon_points)
        polygon_inner = Polygon(inner_polygon_points)
        if polygon_outer.contains(point):
            if polygon_inner.contains(point):
                return False
            else:
                return True
        else:
            return False

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass
    
    def get_vehicle_speed(self, actor):
        vel = actor.get_velocity()
        speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)#km/hr * 1000m/km * 60min/hr * 60sec/min = 3.6 m/s 
        return speed

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.ego,
            self.Adversary_3,
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
        actor_type = get_actor_display_name(event.other_actor)
        #print('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
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
        #print(center_point_y)
        center_point_x = self.top - self.box_height*(i) - self.box_height/2
        #print(center_point_x)
        location = carla.Location(x=center_point_x,y=center_point_y,z=1)
        #print(location)
        if(draw_time > 0): self.draw_location_on_grid(location, draw_time)
        return location
    
    def return_grid_from_location(self, location):
        for index in range(1,20):
            if(location.x < self.top - self.box_height * (index-1)): i=index
            if(location.y > self.left + self.box_width* (index-1)): j=index
        
        return (i,j)

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


def categorical_game_loop(args):
    """
    Main loop of the simulation.
    """
    world = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        sim_world = client.get_world()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            if(args.no_render): settings.no_rendering_mode = True
            sim_world.apply_settings(settings)

        grid = Grid(client.get_world(),-65,-100,-10,30)#, draw_time = 60)   
        world = World(client.get_world(), grid, args)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))

        
               
        #read the path the Adversary is supposed to take from the file
        with open(args.file) as f:
            lines = f.readlines()
        point_array=[]
        destination_array=[]
        for line in lines:
            sp = line.split(',')
            point_array.append(sp[1])
        for point in point_array:
            i = math.floor(int(point)/20)
            j = int(point) % 20
            dest = grid.return_location_from_grid(i,j)
            destination_array.append(dest)
        
        
        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        Adversary3_blueprint=blueprints.filter("diamondback")[0]
        Adversary3_blueprint.set_attribute('role_name', 'Adversary3')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        Adversary3_spawn_point = carla.Transform(destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        world.Adversary_3 = world.world.try_spawn_actor(Adversary3_blueprint,Adversary3_spawn_point)
        #grid.draw_location_on_grid(destination_array[0], draw_time = 5)
        world.modify_vehicle_physics(world.Adversary_3)
        
        #ego_spawn_point = carla.Transform(grid.return_location_from_grid(8,20),carla.Rotation(roll=0,pitch=0,yaw=-90))
        #world.ego = world.world.try_spawn_actor(ego_blueprint,ego_spawn_point)
        world.modify_vehicle_physics(world.ego)   
        world.collision_sensor = CollisionSensor(world.Adversary_3)    
        
        if world._args.sync:
            world.world.tick()
        else:
            world.world.wait_for_tick()
        
        #now intialize the agents
        dest_index = 1
        Adversary3_agent = SimpleAgent(world.Adversary_3, destination_array[dest_index], target_speed=8)
        #grid.draw_location_on_grid(destination_array[1], draw_time = 5)
                
        #ego_agent = BasicAgent(world.ego, target_speed = 7)
        #ego_dest = carla.Location(x=-40,y=0,z=0)
        #waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
        #ego_agent.set_destination(waypoint.transform.location)
        
        Adv3_dest = grid.return_location_from_grid(16,14)

        global PATH_DISTANCE
        if(destination_array[len(destination_array)-1] != Adv3_dest):
            PATH_DISTANCE += 999
            print(f"{PATH_DISTANCE:8.6f}")
            return 0
        
        while True:
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            

            Adversary3_speed = world.get_vehicle_speed(world.Adversary_3)
            #ego_speed = world.get_vehicle_speed(world.ego)    

            #ego_loca = world.ego.get_location()
            Adversary3_loca = world.Adversary_3.get_location()
            
            #distance = math.sqrt((ego_loca.x - Adversary1_loca.x)**2 + (ego_loca.y - Adversary1_loca.y)**2)
            #print(f"Distance between the two is {distance:8.4f}")
               
            if(COLLISION):
                #print("Ending simulation due to collision")
                PATH_DISTANCE += 999
                print(f"{PATH_DISTANCE:8.6f}")
                break
            
            if Adversary3_agent.done():
                if args.loop:
                    #print(world.world.get_snapshot().timestamp)
                    #print(f"Adversary 1 has reached destination {dest_index}:\tSpeed was supposed to be {speed_array[dest_index]}, it is actually {Adversary1_speed}")
                    #if(ego_loca.y > grid.left): 
                    #    i,j = grid.return_grid_from_location(world.ego.get_location())
                    #    #print(f"Ego location: ({i},{j}), speed: {ego_speed}")
                    
                    if dest_index == (len(destination_array) - 1):
                        #print("Adversary 3's route is complete")
                        print(f"{PATH_DISTANCE:8.6f}")
                        break
                    dest_index = dest_index+1
                    new_dest = destination_array[dest_index]
                    if(world.is_in_crosswalk(new_dest)):
                        #world.world.debug.draw_point(new_dest,size=0.2,color=carla.Color(255,0,0),life_time=3)
                        PATH_DISTANCE += 0#math.sqrt((new_dest.x - Adversary3_loca.x)**2 + (new_dest.y - Adversary3_loca.y)**2)-1
                    else: 
                        PATH_DISTANCE += math.sqrt((new_dest.x - Adversary3_loca.x)**2 + (new_dest.y - Adversary3_loca.y)**2)
                        #grid.draw_location_on_grid(new_dest, draw_time = 3)
                    Adversary3_agent.set_destination(new_dest)
                    
                    
                    #new_speed =speed_array[dest_index] 
                    #Adversary1_agent.set_target_speed(new_speed)
                                        
                else:
                    #print("The target has been reached, stopping the simulation")
                    break
            
            control = Adversary3_agent.run_step()
            control.manual_gear_shift = False
            world.Adversary_3.apply_control(control)
            #if (ego_loca.y > grid.left):
            #    world.ego.apply_control(ego_agent.run_step())   
            #else:
            #    world.ego.apply_control(ego_agent.add_emergency_stop(carla.VehicleControl()))        
        

    
    finally:
        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            #settings.no_rendering_mode = False
            world.world.apply_settings(settings)
            

            world.destroy()

def normal_game_loop(args):
    """
    Main loop of the simulation.
    """
    world = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        sim_world = client.get_world()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            if(args.no_render): settings.no_rendering_mode = True
            sim_world.apply_settings(settings)

         
        if(args.debug):  grid = Grid(client.get_world(),-65,-100,-10,30, draw_time = 10)
        else: grid = Grid(client.get_world(),-65,-100,-10,30)
        world = World(client.get_world(), grid, args)

        #grid.return_location_from_grid(0,5,60)
        #grid.return_location_from_grid(16,14,60)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))

        #read the path the Adversary is supposed to take from the file
        with open(args.file) as f:
            lines = f.readlines()
        point_array=[]
        speed_array=[]
        destination_array=[]
        for line in lines:
            sp = line.split(',')
            point_array.append(sp[1])
            if ("NaN" in sp[2] or float(sp[2])<0): speed_array.append(-999.000)
            else: 
                speed = float(sp[2]) * 3.6# * 10
                #if(speed < 1.0): speed_array.append(10.0)
                #else: speed_array.append(speed) 
                speed_array.append(speed)
        for point in point_array:
            i = math.floor(int(point)/20)
            j = int(point) % 20
            dest = grid.return_location_from_grid(i,j)
            destination_array.append(dest)
        
        
        
        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        Adversary3_blueprint=blueprints.filter("diamondback")[0]
        Adversary3_blueprint.set_attribute('role_name', 'Adversary3')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        Adversary3_spawn_point = carla.Transform(destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        world.Adversary_3 = world.world.try_spawn_actor(Adversary3_blueprint,Adversary3_spawn_point)
        if(args.debug): grid.draw_location_on_grid(destination_array[0], draw_time = 5)
        world.modify_vehicle_physics(world.Adversary_3)
        
        ego_spawn_point = carla.Transform(grid.return_location_from_grid(8,20),carla.Rotation(roll=0,pitch=0,yaw=-90))
        world.ego = world.world.try_spawn_actor(ego_blueprint,ego_spawn_point)
        world.modify_vehicle_physics(world.ego)   
        world.collision_sensor = CollisionSensor(world.Adversary_3)    
        
        if world._args.sync:
            world.world.tick()
        else:
            world.world.wait_for_tick()
        
        #now intialize the agents
        dest_index = 1
        Adversary3_agent = SimpleAgent(world.Adversary_3, destination_array[dest_index], target_speed=8)
        if(args.debug): grid.draw_location_on_grid(destination_array[1], draw_time = 5)
                
        ego_agent = BasicAgent(world.ego, target_speed = 20,  opt_dict={'ignore_traffic_lights':'True'})
        ego_dest = grid.return_location_from_grid(8,0)
        waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
        ego_agent.set_destination(waypoint.transform.location)
        if(args.debug): grid.draw_location_on_grid(waypoint.transform.location)
        
        
        for spd in speed_array:
            if spd < 0:
                global TIME_DIFF
                TIME_DIFF = 999.000
                print(f"{TIME_DIFF:8.6f}")
                return 0

        global EGO_DONE
        EGO_DONE = False
        
        while True:
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            

            Adversary3_speed = world.get_vehicle_speed(world.Adversary_3)
            ego_speed = world.get_vehicle_speed(world.ego)    

            ego_loca = world.ego.get_location()
            Adversary3_loca = world.Adversary_3.get_location()
            distance = math.sqrt((ego_loca.x - Adversary3_loca.x)**2 + (ego_loca.y - Adversary3_loca.y)**2)
            #print(f"Distance between the two is {distance:8.4f}")
               
            if(COLLISION):
                #print("Ending simulation due to collision")
                TIME_DIFF = -1
                #print(f"Minimum distance was: {MIN_DISTANCE}")
                print(f"{TIME_DIFF:8.6f}")
                break


            if ego_agent.is_done(waypoint.transform.location) and not EGO_DONE:
                ego_done_time = world.world.get_snapshot().timestamp.elapsed_seconds
                #print(ego_done_time)
                EGO_DONE = True
            
            if Adversary3_agent.done():
                if args.loop:
                    #print(world.world.get_snapshot().timestamp)
                    #print(f"Adversary 1 has reached destination {dest_index}:\tSpeed was supposed to be {speed_array[dest_index]}, it is actually {Adversary1_speed}")
                    #if(ego_loca.y > grid.left): 
                    #    i,j = grid.return_grid_from_location(world.ego.get_location())
                    #    #print(f"Ego location: ({i},{j}), speed: {ego_speed}")
                    
                    if dest_index == (len(destination_array) - 1):
                        #print("Adversary 3's route is complete")
                        #print(f"{PATH_DISTANCE:8.6f}")
                        if(not EGO_DONE):
                            TIME_DIFF = 100
                        else:
                            Adv3_done_time = world.world.get_snapshot().timestamp.elapsed_seconds
                            TIME_DIFF = Adv3_done_time - ego_done_time
                        print(f"{TIME_DIFF:8.6f}")
                        break
                    dest_index = dest_index+1
                    new_dest = destination_array[dest_index]
                    if(args.debug):
                        if(world.is_in_crosswalk(new_dest)):
                            world.world.debug.draw_point(new_dest,size=0.2,color=carla.Color(0,0,255),life_time=3)
                        else: 
                            grid.draw_location_on_grid(new_dest, draw_time = 3)
                    
                    Adversary3_agent.set_destination(new_dest)
                    new_speed =speed_array[dest_index] 
                    Adversary3_agent.set_target_speed(new_speed)
                                        
                else:
                    #print("The target has been reached, stopping the simulation")
                    break
            
            control = Adversary3_agent.run_step()
            control.manual_gear_shift = False
            world.Adversary_3.apply_control(control)
            world.ego.apply_control(ego_agent.run_step())     
        

    
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
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
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

    

    log_level = logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    #logging.info('listening to server %s:%s', args.host, args.port)
    #print(__doc__)

    
    try:
        global COLLISION
        COLLISION = False
        global TIME_DIFF
        TIME_DIFF = 0.0
        global PATH_DISTANCE
        PATH_DISTANCE = 0.0
        if "Categorical" in args.file:
            categorical_game_loop(args)
        else:
            normal_game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()