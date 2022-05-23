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
        
        
        
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-40,y=13,z=15),carla.Rotation(roll=0,pitch=-90,yaw=0)))
            
        
        waypoint = world.get_map().get_waypoint(carla.Location(x=-41,y=45,z=0),project_to_road=True, lane_type=(carla.LaneType.Driving))
        #print(waypoint.transform)
        world.debug.draw_point(waypoint.transform.location, size=1, color=carla.Color(255,0,0), life_time=10)
        
        world.debug.draw_line(carla.Location(x=-33,y=3,z=1), carla.Location(x=-62,y=3,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-33,y=5,z=1), carla.Location(x=-62,y=5,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-33,y=7,z=1), carla.Location(x=-62,y=7,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-33,y=9,z=1), carla.Location(x=-62,y=9,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-33,y=11,z=1), carla.Location(x=-62,y=11,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-33,y=13,z=1), carla.Location(x=-62,y=13,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
   
        
        world.debug.draw_string(carla.Location(x=-32,y=3,z=1),'0',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-32,y=5,z=1),'1',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-32,y=7,z=1),'2',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-32,y=9,z=1),'3',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-32,y=11,z=1),'4',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)


        world.debug.draw_line(carla.Location(x=-33,y=3,z=1), carla.Location(x=-33,y=33,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-35,y=3,z=1), carla.Location(x=-35,y=33,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-37,y=3,z=1), carla.Location(x=-37,y=33,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-39,y=3,z=1), carla.Location(x=-39,y=33,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)
        world.debug.draw_line(carla.Location(x=-41,y=3,z=1), carla.Location(x=-41,y=33,z=1), thickness=0.1, color=carla.Color(255,0,0), life_time=10)

        world.debug.draw_string(carla.Location(x=-33,y=2,z=1),'0',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-35,y=2,z=1),'1',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-37,y=2,z=1),'2',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-39,y=2,z=1),'3',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        world.debug.draw_string(carla.Location(x=-41,y=2,z=1),'4',draw_shadow=False,color=carla.Color(0,255,0),life_time=10)
        
        print("Ticking the world\n")
        current_tick = world.tick()
           
        
        
        while (True):
            #Do whatever you want here *******************
            #*********************************************
            s = world.get_snapshot().timestamp
            
            #*********************************************
            #Do whatever you want here *******************
            current_tick = world.tick()
        
    finally:
        print("Finally:\n")
        
        #Always disable sync mode before the script ends to prevent the server blocking while waiting for a tick
        
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        
               
        

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with code.')
