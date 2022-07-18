#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    7/18/2022
# Purpose:          Main script for running/retrieving data from simulation for Cross-Entropy

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
        self.collision_sensor = None
        self.obstacle_sensor_adv = None
        self.obstacle_sensor_ego = None
        self.feature_vector = []
    
    def write_features(self, score,args):
        file_path = "C:\\data\\Features\\"
        if(score < 0):
            num = args.file.split('#')[1]
            file_name = f"features_{num}_BAD_{score:8.6f}"
            file = file_path + file_name + ".csv"
            with open(file,"w",newline="") as f:
                writer = csv.writer(f)
                writer.writerows(self.feature_vector)
        elif(score >= 0 and score < 500):
            num = args.file.split('#')[1]
            file_name = f"features_{num}_GOOD_{score:8.6f}"
            file = file_path + file_name + ".csv"
            with open(file,"w",newline="") as f:
                writer = csv.writer(f)
                writer.writerows(self.feature_vector)
        else: 
            return
    
    def get_features(self):
        snapshot = self.world.get_snapshot()
        frame = snapshot.frame
        actors = []
        ActorSnapshot_ego = snapshot.find(self.Adversary.id)
        actors.append(ActorSnapshot_ego)
        ActorSnapshot_adv = snapshot.find(self.ego.id)
        actors.append(ActorSnapshot_adv)

        frame_feature_vector = []
        frame_feature_vector.append(frame)
        for actor in actors:
            frame_feature_vector.append(actor.get_transform().location.x)
            frame_feature_vector.append(actor.get_transform().location.y)
            frame_feature_vector.append(actor.get_transform().location.z)
            
            frame_feature_vector.append(actor.get_transform().rotation.roll)
            frame_feature_vector.append(actor.get_transform().rotation.pitch)
            frame_feature_vector.append(actor.get_transform().rotation.yaw)

            frame_feature_vector.append(actor.get_velocity().x)
            frame_feature_vector.append(actor.get_velocity().y)
            frame_feature_vector.append(actor.get_velocity().z)

            frame_feature_vector.append(actor.get_angular_velocity().x)
            frame_feature_vector.append(actor.get_angular_velocity().y)
            frame_feature_vector.append(actor.get_angular_velocity().z)

            frame_feature_vector.append(actor.get_acceleration().x)
            frame_feature_vector.append(actor.get_acceleration().y)
            frame_feature_vector.append(actor.get_acceleration().z)
            
        self.feature_vector.append(frame_feature_vector)
    
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
            self.Adversary,
            self.collision_sensor.sensor,
            self.obstacle_sensor_adv.sensor,
            self.obstacle_sensor_ego.sensor]
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


# ==============================================================================
# -- ObstacleSensor -----------------------------------------------------------
# ==============================================================================
class ObstacleSensor(object):
    """ Class for obstacle sensors"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self.history = []
        self.speeds = []
        self._parent = parent_actor
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.obstacle')
        blueprint.set_attribute('only_dynamics','True')
        blueprint.set_attribute('distance','10')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: ObstacleSensor._on_obstacle_detected(weak_self, event))

    def get_smallest_distance(self):
        """Gets the closest distance between Adversary and ego"""
        if(len(self.history)>0): return sorted(self.history)[0]
        else: return 999
    
    def get_shortest_ttc(self):
        
        if(len(self.history)>0):
            ttcs = []
            for i in range(0,len(self.history)):
                ttc = self.history[i] / self.speeds[i]
                ttcs.append(ttc)
            return sorted(ttcs)[0]
        else: return 100

    @staticmethod
    def _on_obstacle_detected(weak_self, event):
        """On obstacle_detect method"""
        self = weak_self()
        if not self:
            return
        self.history.append(event.distance)
        vel = self._parent.get_velocity()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) #m/s
        self.speeds.append(speed)
        
        

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
    score = 0

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        sim_world = client.get_world()

        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        if(args.no_render): settings.no_rendering_mode = True
        sim_world.apply_settings(settings)

        
        """
        if(args.debug): grid = Grid(client.get_world(),-33,-61,4,38, draw_time = 30)
        else: grid = Grid(client.get_world(),-33,-61,4,38)     
        world = World(client.get_world(), grid, args)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-47,y=21,z=30),carla.Rotation(roll=0, pitch=-90,yaw=0)))
        """

        if(args.debug): grid = Grid(client.get_world(),-65,-100,-10,30, draw_time = 60)
        else: grid = Grid(client.get_world(),-65,-100,-10,30)   
        world = World(client.get_world(), grid, args)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))
        #spectator.set_transform(carla.Transform(carla.Location(x=-75,y=7,z=20),carla.Rotation(roll=0, pitch=-90,yaw=0)))
        

        #read the path the Adversary is supposed to take from the file
        with open(args.file) as f:
            lines = f.readlines()
        point_array=[]
        speed_array=[]
        accel_array=[]
        destination_array=[]
        for line in lines:
            sp = line.split(',')
            point_array.append(sp[1])
            speed_array.append(float(sp[2]) * 3.6) #converts velocity in m/s to km/hr
            accel_array.append(float(sp[3]) * 3.6) #converts accel in m/(s * s) to km/(hr * s)
        for point in point_array:
            i, j = grid.return_coords_from_point(point) 
            dest = grid.return_location_from_grid(i,j)
            destination_array.append(dest)
        
        SpeedorAccel = None
        if(len(speed_array) > 1):
            if(speed_array[1] == accel_array[1]): SpeedorAccel = "Speed"
            else: SpeedorAccel = "Accel"

             
        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        Adversary1_blueprint=blueprints.filter("diamondback")[0]
        Adversary1_blueprint.set_attribute('role_name', 'Adversary1')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        Adversary_spawn_point = carla.Transform(destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        world.Adversary = world.world.try_spawn_actor(Adversary1_blueprint,Adversary_spawn_point)
        """
        if(args.debug):
            i,j = grid.return_grid_from_location(Adversary_spawn_point.location)
            point = grid.return_point_from_coords(i,j)
            print(f"Adversary is located at ({Adversary_spawn_point.location.x},{Adversary_spawn_point.location.y}), grid: ({i},{j}) -> {point}") 
            grid.draw_location_on_grid(Adversary_spawn_point.location, draw_time = 5)
        """
        if(args.debug): grid.draw_location_on_grid(Adversary_spawn_point.location, draw_time = 5)
        if(args.debug_nums):
            world.world.debug.draw_point(Adversary_spawn_point.location,size=0.2,color=carla.Color(255,255,255),life_time=100)
            #world.world.debug.draw_string(Adversary_spawn_point.location+carla.Location(z=.5),"START",draw_shadow=True,color=carla.Color(0,0,255), life_time=60)
        
        
        ego_spawn_point = carla.Transform(grid.return_location_from_grid(8,20),carla.Rotation(roll=0,pitch=0,yaw=-90))
        world.ego = world.world.try_spawn_actor(ego_blueprint,ego_spawn_point)

        world.collision_sensor = CollisionSensor(world.Adversary)    
        world.obstacle_sensor_adv = ObstacleSensor(world.Adversary)
        world.obstacle_sensor_ego = ObstacleSensor(world.ego)

        #This is necessary to ensure vehicle "stabilizes" after "falling"
        for i in range (0,30):
            world.world.tick()

        #No negative speed, no NaNs
        for spd in speed_array:
            if math.isnan(spd) or spd < 0:
                score += 999
                return 0

        adv = None
        if "Normal" in args.file:
            adv = Adversary(point_array, speed_array, accel_array, destination_array, -1) 
            simulate_normal_distribution(world, adv, args, SpeedorAccel)
            score += adv.cost
        if "Categorical" in args.file:
            adv = Adversary(point_array, speed_array, accel_array, destination_array, -10) 
            simulate_categorical_distribution(world, adv, args)
            score += adv.cost

    finally:
        print(f"{score:8.6f}")

        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            #settings.no_rendering_mode = False
            world.world.apply_settings(settings)

            if "Normal" in args.file:
                world.write_features(score,args)

            world.destroy()


# ==============================================================================
# -- categorical distribution---------------------------------------------------
# ==============================================================================
def simulate_categorical_distribution(world, adversary, args):
    grid = world._grid
    #now intialize the agents
    dest_index = 1
    Adversary_agent = SimpleAgent(world.Adversary, adversary.dest_array[dest_index], target_speed=20)
    if(args.debug): grid.draw_location_on_grid(adversary.dest_array[1], draw_time = 5)
    if(args.debug_nums): world.world.debug.draw_point(adversary.dest_array[1],size=0.2,color=carla.Color(255,255,255),life_time=100)
    
    Adv3_dest = grid.return_location_from_grid(16,14)

    if(adversary.dest_array[len(adversary.dest_array)-1] != Adv3_dest):
        adversary.cost += 999
        return 0
    
    adversary.cost += -1/len(adversary.dest_array)
    
    while True:
        world.world.tick()        

        Adversary_speed = world.get_vehicle_speed(world.Adversary)
        Adversary_loca = world.Adversary.get_location()

        if(len(world.collision_sensor.get_collision_history())>0):
            #print("Ending simulation due to collision")
            adversary.cost += 999
            break
        
        if Adversary_agent.done():
            if (dest_index >= len(adversary.dest_array)-1):
                #print("Adversary's route is complete, breaking out of loop")
                if(args.debug_nums): print(len(adversary.dest_array))
                break
            else:
                dest_index = dest_index+1
                new_dest = adversary.dest_array[dest_index]
                if(world.is_in_crosswalk(new_dest)):
                    if(args.debug): world.world.debug.draw_point(new_dest,size=0.2,color=carla.Color(255,0,0),life_time=3)
                    if(args.debug_nums): world.world.debug.draw_point(new_dest,size=0.2,color=carla.Color(0,0,255),life_time=100)
                    adversary.cost += 0
                else: 
                    adversary.cost += .1 #1/world.get_2D_distance(Adversary_loca,new_dest)
                    if(args.debug): grid.draw_location_on_grid(new_dest, draw_time = 3)
                    if(args.debug_nums): world.world.debug.draw_point(new_dest,size=0.2,color=carla.Color(255,255,255),life_time=100)
                        
                
                Adversary_agent.set_destination(new_dest)

        
        control = Adversary_agent.run_step()
        control.manual_gear_shift = False
        world.Adversary.apply_control(control)


# ==============================================================================
# -- normal distribution--------------------------------------------------------
# ==============================================================================
def simulate_normal_distribution(world, adversary, args, SpeedorAccel):
    cost_funct = "TTC"#either "TTC" or "finish_time"
    grid = world._grid
    #now intialize the agents
    dest_index = 1
    destination = adversary.dest_array[dest_index]
    Adversary_loca = world.Adversary.get_location()
    if SpeedorAccel == "Speed": target_speed = adversary.speed_array[dest_index]
    else: 
        distance = world.get_2D_distance(Adversary_loca,destination)    
        target_speed = 0 + adversary.accel_array[dest_index-1] * (distance/10.0 * 3.6)  
    Adversary_agent = SimpleAgent(world.Adversary, destination, target_speed=target_speed)
    if(args.debug): grid.draw_location_on_grid(adversary.dest_array[1], draw_time = 5)

    """        
    ego_agent = BasicAgent(world.ego, target_speed = 7)
    ego_dest = carla.Location(x=-40,y=0,z=0)
    waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
    ego_agent.set_destination(waypoint.transform.location)
    """

    ego_agent = BasicAgent(world.ego, target_speed = 20,  opt_dict={'ignore_traffic_lights':'True'})
    #ego_agent = BasicAgent(world.ego, target_speed = 15,  opt_dict={'ignore_traffic_lights':'True'})
    ego_dest = grid.return_location_from_grid(8,0)
    waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
    ego_agent.set_destination(waypoint.transform.location)
    if(args.debug): grid.draw_location_on_grid(waypoint.transform.location)
    ego_done = False
    ego_done_time = 0
    adv_done_time = 0

    
    stuck_counter = 0
    while True:
        world.get_features()
        world.world.tick()
        stuck_counter += 1
        
        Adversary_speed = world.get_vehicle_speed(world.Adversary)
        ego_speed = world.get_vehicle_speed(world.ego)    

        ego_loca = world.ego.get_location()
        Adversary_loca = world.Adversary.get_location()
        adv_ego_distance = world.get_2D_distance(ego_loca, Adversary_loca)

        
        #This detects if the Advesary is stuck (i.e. hasn't moved squares for 100 ticks (5 seconds))
        if(stuck_counter % 100 == 0):
            i,j = grid.return_grid_from_location(Adversary_loca)
            pt = grid.return_point_from_coords(i,j)
            #print(f"located at {pt} == {int(adversary.point_array[dest_index - 1])}")
            if(pt == int(adversary.point_array[dest_index - 1])):
                adversary.cost += 999
                return 0
        
        if(len(world.collision_sensor.get_collision_history())>0):
            #print("Ending simulation due to collision")
            adversary.cost += 0
            break
        
        
        if Adversary_agent.done():
            
            """
            if(args.debug): 
                print(world.world.get_snapshot().timestamp)
                i,j = grid.return_grid_from_location(adversary.dest_array[dest_index])
                point = grid.return_point_from_coords(i,j)
                print(f"Adversary has reached destination {dest_index} at point {point}")
                print(f"Target speed was {target_speed:4.2f} but speed is actually {Adversary_speed:4.2f} km/hr")
            """
                                        
            if (dest_index >= len(adversary.dest_array)-1):
                #print("Adversary's route is complete, breaking out of loop")
                adv_done_time = world.world.get_snapshot().timestamp.elapsed_seconds
                if cost_funct == "finish_time":
                    if(not ego_done):
                        #print("Adversary is done, ego not done yet")
                        adversary.cost +=500
                    else:                
                        #print(f"Adversary done at {adv_done_time}, ego done at {ego_done_time}")
                        adversary.cost += adv_done_time - ego_done_time
                elif cost_funct == "TTC":                  
                    adv_ttc = world.obstacle_sensor_adv.get_shortest_ttc()
                    ego_ttc = world.obstacle_sensor_ego.get_shortest_ttc()
                    if(adv_ttc < ego_ttc): 
                        #print(f"TTC is {adv_ttc:8.6f}")
                        adversary.cost += adv_ttc
                    else: adversary.cost += ego_ttc
                else: print("Cost function not defined")
                break
            else:
                dest_index = dest_index+1
                stuck_counter = 0
                new_dest = adversary.dest_array[dest_index]
                if(args.debug): grid.draw_location_on_grid(new_dest, draw_time = 3)
                
                if SpeedorAccel == "Speed": target_speed = adversary.speed_array[dest_index]
                else:
                    current_speed = Adversary_speed
                    distance = world.get_2D_distance(Adversary_loca,new_dest)
                    target_speed = current_speed + adversary.accel_array[dest_index-1] * (distance/current_speed * 3.6)  
                #print(f"target speed ({target_speed:4.2f}) = current speed ({current_speed:4.2f} km/hr) + accel ({adversary.accel_array[dest_index-1]:4.2f} km/(hr*s)) * (distance ({distance:4.2f} meters) / speed ({current_speed:4.2f} km\hr) * 3.6 km*sec/(m*hr))")

                Adversary_agent.set_destination(new_dest)
                Adversary_agent.set_target_speed(target_speed)
                                    
        
        control = Adversary_agent.run_step()
        control.manual_gear_shift = False
        world.Adversary.apply_control(control)
        
        
        
        if(world.get_2D_distance(waypoint.transform.location,ego_loca) < 0.5 and not ego_done):
            #print("Ego_agent is done")
            ego_done_time = world.world.get_snapshot().timestamp.elapsed_seconds
            ego_done =  True

        elif(not ego_done):
            world.ego.apply_control(ego_agent.run_step())
        
        """
        if (ego_loca.y > (grid.left-10)):
            world.ego.apply_control(ego_agent.run_step())   
        else:
            world.ego.apply_control(ego_agent.add_emergency_stop(carla.VehicleControl()))
        """  

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