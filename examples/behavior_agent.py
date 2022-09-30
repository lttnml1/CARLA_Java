#!/usr/bin/env python

#**********************************************************************
#   Author: Matthew Litton
#   Last Modified: 9/9/2022
#   Purpose: collecting data on vehicle behavior
#**********************************************************************



import time
import numpy as np
import argparse
import sys
import glob
import os
import math
from numpy import random
import logging
import weakref
import pandas as pd
from collections import deque

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.simple_agent import SimpleAgent

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
# ==============================================================================

import carla

def draw_location(world, location, draw_time = 10):
        world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)

def get_2D_distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)

def execute_scenario(args):
    world = None
    ego = None
    vehicles_list = []

    client = carla.Client(args.host,args.port)
    client.set_timeout(4.0)
    try:
        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(0)
           
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))

        blueprints = world.get_blueprint_library()
        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        map = world.get_map()
        spawn_points = map.get_spawn_points()
        ego_spawn_point = random.choice(spawn_points)
        ego = world.try_spawn_actor(ego_blueprint,ego_spawn_point)
        ego_destination = random.choice(spawn_points).location
        
        #random.shuffle(spawn_points)

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, tm.get_port())))

        for response in client.apply_batch_sync(batch, True):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)
        
        print('Spawned %d vehicles, press Ctrl+C to exit.\n' % (len(vehicles_list)))
      

        for i in range(0,30):
            world.tick()

        ego_agent = BehaviorAgent(ego, behavior='normal',opt_dict={'offset':0.0})
        #draw_location(world, ego_destination)
        ego_agent.set_destination(ego_destination)



        spectator.set_transform(ego.get_transform())
        waypoints_queue = ego_agent.get_waypoints() #this is a SHALLOW copy - good which means it can be modified
        
        offset = 1.0
        for i in range(len(waypoints_queue)):
            waypoint = waypoints_queue[i][0]
            road_option = waypoints_queue[i][1]

            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(x=offset*r_vec.x, y=offset*r_vec.y)
            #waypoints_queue[i] = (carla.Transform(carla.Location(w_loc),waypoints_queue[i][0].transform.rotation),road_option)
            draw_location(world,w_loc,240)
            #draw_location(world, waypoint[0].transform.location,240)

            
        
        counter = 0
        while True:
            world.tick()
            
            #spectator.set_transform(carla.Transform(ego.get_transform().location+carla.Location(z=3),ego.get_transform().rotation))
            if ego_agent.done():
                ego_agent.set_destination(random.choice(spawn_points).location)

            ego.apply_control(ego_agent.run_step())
            counter+=1
            
    finally:
        print("Cleaning up\n")
        if world is not None:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

            if ego is not None:
                ego.destroy()
            
            print('\nDestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            tm.set_synchronous_mode(False)
            time.sleep(0.5)

def main():
    program_start_time = time.time()

    argparser = argparse.ArgumentParser()
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
        '-n', '--number-of-vehicles',
        metavar='N',
        default=30,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '--no_render',
        action = 'store_true',
        help='Render graphics (default: False)')
    args = argparser.parse_args()

    try:
        
        execute_scenario(args)
    
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    finally:
        print(f"TOTAL RUN TIME: {time.time()-program_start_time}")
if __name__ == '__main__':
    main()
