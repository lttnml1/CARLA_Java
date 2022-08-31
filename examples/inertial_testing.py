#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    8/31/2022
# Purpose:          

from __future__ import print_function

import argparse
import glob
import math
import os
import numpy.random as random
import sys
import weakref
import pandas

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
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from agents.navigation.simple_agent import SimpleAgent

# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================

def game_loop(args):

    world = None
    ego = None
    imu_sensor = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)
        
        sim_world = client.get_world()
        
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        if(args.no_render): settings.no_rendering_mode = True
        sim_world.apply_settings(settings)
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(0)

        #set the view to the middle of the grid
        spectator = sim_world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-100,y=14,z=50),carla.Rotation(roll=0, pitch=-70,yaw=0)))

        
        # Spawn the actors
        blueprints = sim_world.get_blueprint_library()

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')


        world_map = sim_world.get_map()
        spawn_points = world_map.get_spawn_points()
        
        ego_spawn_point = spawn_points[15]

        ego = sim_world.try_spawn_actor(ego_blueprint, ego_spawn_point)
        ego.set_autopilot(True,8000)
        tm.ignore_lights_percentage(ego,100)
        #tm.vehicle_percentage_speed_difference(world.ego,60)

        imu_sensor = IMUSensor(ego)

        

        #This is necessary to ensure vehicle "stabilizes" after "falling"
        for i in range (0,30):
            sim_world.tick()
        
        counter = 0
        while(True):
            spectator.set_transform(carla.Transform(ego.get_transform().location+carla.Location(z=5),ego.get_transform().rotation))
            sim_world.tick()
            counter+=1
            if(not counter%100):
                ego_accel = ego.get_acceleration()
                #print(f"[{ego_accel.x/9.8:6.4f}, {ego_accel.y/9.8:6.4f}, {ego_accel.z/9.8:6.4f}]")
                
                #convert to g's
                accel_in_gs = tuple(map(lambda x: x/9.8, imu_sensor.accelerometer))
                print(f"\n[{imu_sensor.accelerometer}]")
                print(f"[{accel_in_gs}]\n")


    finally:
        if sim_world is not None:
            #tm.set_synchronous_mode(False)
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            #settings.no_rendering_mode = False
            sim_world.apply_settings(settings)
            if ego is not None:
                if imu_sensor is not None:
                    imu_sensor.sensor.destroy()
                ego.destroy()

            

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

    args = argparser.parse_args()
    
    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()