#!/usr/bin/env python

#**********************************************************************
#   Author: Matthew Litton
#   Last Modified: 9/8/2022
#   Purpose: Cross-Entropy simulation for CARLA
#**********************************************************************

"""
Outline of program

1) Initialize parameters (N, rho, gamma)

2) Run cross-entropy while(g>gamma)
    2a) draw N random samples
    2b) score each random sample
    2c) sort from smallest to largest
    2d) find the elite set (g is the largest in the elite set)
    2e) update the parameters based on the unscored samples that make up the elite set

3) now you have a distribution that is more likely to generate "bad" behavior

"""


import time
import numpy as np
import argparse
import sys
import glob
import os
import math

from cross_entropy import CrossEntropy
from normal_distrib import NormalDistrib

from agents.navigation.basic_agent import BasicAgent
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


def execute_scenario(args, adversary_target_speed):
    world = None
    ego = None
    adv = None
    score = None
    try:
        client = carla.Client(args.host,args.port)
        client.set_timeout(4.0)

        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))

        blueprints = world.get_blueprint_library()
        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')
        adversary_blueprint=blueprints.filter("vehicle.citroen.c3")[0]
        adversary_blueprint.set_attribute('role_name', 'adversary')

        map = world.get_map()
        spawn_points = map.get_spawn_points()
        ego_spawn_point = spawn_points[228]
        adversary_spawn_point = spawn_points[61]
        #adversary_spawn_point = carla.Transform(carla.Location(x=-67.668266, y=5.793464, z=0.275),carla.Rotation(roll=0,pitch=0,yaw=-160))
        #adversary_spawn_point = carla.Transform(carla.Location(x=-82.668266, y=8.793464, z=0.275),carla.Rotation(roll=0,pitch=0,yaw=-160))
        #draw_location(world, ego_spawn_point.location)
        ego = world.try_spawn_actor(ego_blueprint,ego_spawn_point)
        adv = world.try_spawn_actor(adversary_blueprint,adversary_spawn_point)

   

        for i in range(0,30):
            world.tick()
        
        ego_agent = BasicAgent(ego, target_speed = 10,opt_dict={'ignore_traffic_lights':'True','base_vehicle_threshold':20.0})
        ego_destination = spawn_points[31].location
        #draw_location(world, ego_destination)
        ego_agent.set_destination(ego_destination)
        #adv_destination = carla.Location(x=-94.079308, y=24.415840, z=0.031020)
        adv_destination = spawn_points[31].location
        adv_agent = SimpleAgent(adv, adv_destination, target_speed = adversary_target_speed * 3.6)

        score = get_2D_distance(adversary_spawn_point.location,ego_spawn_point.location)

        counter = 0
        score_frame = 0
        bounding_boxes = []
        while True:
            world.tick()
            world_snapshot = world.get_snapshot()
            frame = world_snapshot.timestamp.frame

            dist = get_2D_distance(ego.get_transform().location,adv.get_transform().location)
            if(dist<score): 
                score=dist
                score_frame = frame
                bounding_boxes = []
                for actor_snapshot in world.get_actors().filter('*vehicle*'):
                    actual_actor = world.get_actor(actor_snapshot.id)
                    bb = [actual_actor.bounding_box.extent.x,actual_actor.bounding_box.extent.y,actual_actor.bounding_box.extent.z]
                    #world.debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(bb[0],bb[1],bb[2])),actor_snapshot.get_transform().rotation,0.1,carla.Color(0,255,0),30)
                    bounding_boxes.append((carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(bb[0],bb[1],bb[2])),actor_snapshot.get_transform().rotation))
            ego_done = ego_agent.done()
            adv_done = adv_agent.done()
            if(ego_done or adv_done): break
            else:
                ego.apply_control(ego_agent.run_step())
                adv.apply_control(adv_agent.run_step())
            counter +=1
             
            
    finally:
        #print(score_frame, bounding_boxes)
        for box in bounding_boxes:
            world.debug.draw_box(box[0],box[1],0.1,carla.Color(0,255,0),1)
        if world is not None:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

            if ego is not None:
                ego.destroy()
            if adv is not None:
                adv.destroy()
        return score

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
    args = argparser.parse_args()

    try:
        distributions = []
        adversary_target_speed = NormalDistrib(2,1)
        distributions.append(adversary_target_speed)
        
        ce = CrossEntropy(10,0.1,5,distributions)
        y = ce.draw_random_samples()
        scores = np.empty(y.shape)
        for i in range(np.shape(y)[0]):
            scores[i,0] = execute_scenario(args, y[i,0])
        
        ce.calculate_elite(y, scores)
            
    
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    finally:
        print(f"TOTAL RUN TIME: {time.time()-program_start_time}")

if __name__ == '__main__':
    main()
