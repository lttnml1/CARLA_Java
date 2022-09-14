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

# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
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
            max(limits[0], min(limits[1], sensor_data.accelerometer.z/9.8)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
# ==============================================================================

def draw_location(world, location, draw_time = 10):
        world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)

def get_2D_distance(loc1, loc2):
    return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)

def execute_scenario(args):
    world = None
    ego = None
    imu_sensor = None
    vehicles_list = []

    client = carla.Client(args.host,args.port)
    client.set_timeout(4.0)
    try:
        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        if(args.no_render): settings.no_rendering_mode = True
        world.apply_settings(settings)

        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(0)

        if(not args.no_render):            
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
        imu_sensor = IMUSensor(ego)
        
        random.shuffle(spawn_points)

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
        
        print('spawned %d vehicles, press Ctrl+C to exit.' % (len(vehicles_list)))
      

        for i in range(0,30):
            world.tick()

        ego_agent = BehaviorAgent(ego, behavior='normal')
        #draw_location(world, ego_destination)
        ego_agent.set_destination(ego_destination)
        

        big_data = []
        counter = 0
        while True:
            world.tick()
            if(not args.no_render):  
                spectator.set_transform(carla.Transform(ego.get_transform().location+carla.Location(z=5),ego.get_transform().rotation))
            if ego_agent.done():
                ego_agent.set_destination(random.choice(spawn_points).location)
            ego.apply_control(ego_agent.run_step())
            counter+=1
            if(not counter%10):
                ts = math.floor(world.get_snapshot().timestamp.elapsed_seconds)
                accel = imu_sensor.accelerometer
                gyro = imu_sensor.gyroscope
                data = [accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],'normal',ts]
                big_data.append(data)
            
    finally:
        print("Writing file\n")
        write_data(big_data)
        if world is not None:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

            if ego is not None:
                if imu_sensor is not None:
                    imu_sensor.sensor.destroy()
                ego.destroy()
            
            print('\ndestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            tm.set_synchronous_mode(False)
            time.sleep(0.5)

def write_data(data):
    headers = ['AccX','AccY','AccZ','GyroX','GyroY','GyroZ','Class','Timestamp']
    df = pd.DataFrame(data,columns=headers)
    df.to_csv('C:\\Users\\m.litton_local\\data_analysis\\carla\\normal.csv',index=False)

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
