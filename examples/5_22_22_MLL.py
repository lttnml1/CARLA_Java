#!/usr/bin/env python

# Author: Matt Litton
# Latest Change: 5MAY22
# Purpose: 

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import logging
import math

def main():
    print("Main:\n")
    
    
    client = carla.Client('localhost',2000)
    client.set_timeout(10.0)
    vehicles_list = []
    try:
        print("Try:\n")
        world = client.get_world()
        
        #Put the world in synchronous mode with a fixed time step of .05s (20 FPS)
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)
        
        
        #Car1: Red Tesla Model3
        Car1_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        Car1_bp.set_attribute('color',"255,0,0")

        #Car2: Blue Ambulance
        Car2_bp = world.get_blueprint_library().find('vehicle.ford.ambulance')
        Car2_bp.set_attribute('color',"0,0,255")
                               
        #Get available spawn points
        spawn_points = world.get_map().get_spawn_points()
        Car1_spawn_point = carla.Transform(carla.Location(x=-41.679832, y=45.001911, z=0.600000),carla.Rotation(pitch=0.000000, yaw=-90.161232, roll=0.000000))
        Car2_spawn_point = spawn_points[0]

        batch = []

        batch.append(carla.command.SpawnActor(Car1_bp, Car1_spawn_point))
        batch.append(carla.command.SpawnActor(Car2_bp, Car2_spawn_point))
        for response in client.apply_batch_sync(batch, True):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        for vics in vehicles_list:
            print(vics)
        
        #Car1 = world.try_spawn_actor(Car1_bp,Car1_spawn_point)
        #Car1.enable_constant_velocity(carla.Vector3D(5,0,0))        
        
        waypoint = world.get_map().get_waypoint(carla.Location(x=-41,y=45,z=0),project_to_road=True, lane_type=(carla.LaneType.Driving))
        print(waypoint)
        #world.debug.draw_point(waypoint.transform.location, size=1, color=carla.Color(255,0,0), life_time=0.0)
        

        #print("Ticking the world\n")
        #current_tick = world.tick()
           
        
        #This is the main simulation loop
        min_y = 0
        #y = Car1_spawn_point.location.y
        while (True):
            #Do whatever you want here *******************
            #*********************************************
            s = world.get_snapshot().timestamp
            #y = Car1.get_location().y
            #print(f"Current Tick {current_tick}, Speed is: {Car1.get_velocity().y} m/s, Y is: {y:4.2f} \n")
            #*********************************************
            #Do whatever you want here *******************
            #current_tick = world.tick()
        
    finally:
        print("Finally:\n")
        
        #Always disable sync mode before the script ends to prevent the server blocking while waiting for a tick
        
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        
        print('Destroying vehicles')
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        
        

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with code.')
