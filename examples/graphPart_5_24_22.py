#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    5/24/2022
# Purpose:          Ego goes from right to left, Adv1 goes from bottom to top at normally distributed speed

"""Example of automatic vehicle control from client side."""

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
        self.Adversary_1 = None
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

    def return_location_from_grid(self, i: int, j: int, draw: bool = False):
        center_point_y = self.left + self.box_width*(j-1) + self.box_width/2
        #print(center_point_y)
        center_point_x = self.top - self.box_height*(i-1) - self.box_height/2
        #print(center_point_x)
        location = carla.Location(x=center_point_x,y=center_point_y,z=1)
        #print(location)
        if(draw): self.draw_location_on_grid(location)
        return location
    
    def return_grid_from_location(self, location):
        for index in range(1,20):
            if(location.x < self.top - self.box_height * (index-1)): i=index
            if(location.y > self.left + self.box_width* (index-1)): j=index
        
        return (i,j)

    def draw_location_on_grid(self, location, draw_time = 10):
        self.world.debug.draw_point(location,size=0.5,color=carla.Color(0,255,0),life_time=draw_time)
            
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

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            if(args.no_render): settings.no_rendering_mode = True
            sim_world.apply_settings(settings)

        grid = Grid(client.get_world(),-33,-61,4,38)#, draw_time = 10)   
        world = World(client.get_world(), grid, args)

        #set the view to the middle of the grid
        spectator = world.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=-47,y=21,z=20),carla.Rotation(roll=0, pitch=-90,yaw=0)))
        
        #read the path the Adversary is supposed to take from the file
        with open(args.file) as f:
            lines = f.readlines()
        point_array=[]
        speed_array=[]
        destination_array=[]
        for line in lines:
            sp = line.split(',')
            point_array.append(sp[1])
            speed = float(sp[2]) * 3.6
            if(speed < 1.0): speed_array.append(1.0)
            else: speed_array.append(speed) 
        for point in point_array:
            i = math.floor(int(point)/20)
            j = int(point) % 20
            dest = grid.return_location_from_grid(i,j)
            destination_array.append(dest)
        
        
        # Spawn the actors
        blueprints = world.world.get_blueprint_library()

        Adversary1_blueprint=blueprints.filter("diamondback")[0]
        Adversary1_blueprint.set_attribute('role_name', 'Adversary1')

        ego_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        ego_blueprint.set_attribute('role_name','ego')

        Adversary1_spawn_point = carla.Transform(destination_array[0],carla.Rotation(roll=0,pitch=0,yaw=0))
        world.Adversary_1 = world.world.try_spawn_actor(Adversary1_blueprint,Adversary1_spawn_point)
        #grid.draw_location_on_grid(destination_array[0], draw_time = 20)
        #print(f"Adversary 1 has been spawned {world.Adversary_1}")
        world.modify_vehicle_physics(world.Adversary_1)
        
        ego_spawn_point = carla.Transform(grid.return_location_from_grid(8,20),carla.Rotation(roll=0,pitch=0,yaw=-90))
        world.ego = world.world.try_spawn_actor(ego_blueprint,ego_spawn_point)
        #print(f"Ego has been spawned {world.ego}")
        world.modify_vehicle_physics(world.ego)   
        world.collision_sensor = CollisionSensor(world.Adversary_1)    
        
        if world._args.sync:
            world.world.tick()
        else:
            world.world.wait_for_tick()
        
        #now intialize the agents
        dest_index = 1
        Adversary1_agent = SimpleAgent(world.Adversary_1, destination_array[dest_index], target_speed=speed_array[dest_index])
        #grid.draw_location_on_grid(destination_array[1], draw_time = 20)
                
        ego_agent = BasicAgent(world.ego, target_speed = 7)
        ego_dest = carla.Location(x=-40,y=0,z=0)
        waypoint = world.world.get_map().get_waypoint(ego_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
        #print(waypoint)
        ego_agent.set_destination(waypoint.transform.location)
        
        
        while True:
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            

            Adversary1_speed = world.get_vehicle_speed(world.Adversary_1)
            ego_speed = world.get_vehicle_speed(world.ego)    

            ego_loca = world.ego.get_location()
            Adversary1_loca = world.Adversary_1.get_location()
            distance = math.sqrt((ego_loca.x - Adversary1_loca.x)**2 + (ego_loca.y - Adversary1_loca.y)**2)
            #print(f"Distance between the two is {distance:8.4f}")
            
            global MIN_DISTANCE
            if(distance < MIN_DISTANCE):
                MIN_DISTANCE = distance
            if(COLLISION):
                #print("Ending simulation due to collision")
                MIN_DISTANCE = -1
                #print(f"Minimum distance was: {MIN_DISTANCE}")
                print(f"{MIN_DISTANCE:8.6f}")
                break

            if Adversary1_agent.done():
                if args.loop:
                    #print(world.world.get_snapshot().timestamp)
                    #print(f"Adversary 1 has reached destination {dest_index}:\tSpeed was supposed to be {speed_array[dest_index]}, it is actually {Adversary1_speed}")
                    if(ego_loca.y > grid.left): 
                        i,j = grid.return_grid_from_location(world.ego.get_location())
                        #print(f"Ego location: ({i},{j}), speed: {ego_speed}")                                
                    if dest_index == (len(destination_array) - 2):
                        #print("Adversary 1's route is complete")
                        #print(f"Minimum distance was: {MIN_DISTANCE:8.6f}")
                        print(f"{MIN_DISTANCE:8.6f}")
                        break
                    dest_index = dest_index+1
                    new_dest = destination_array[dest_index]
                    #grid.draw_location_on_grid(new_dest, draw_time = 10)
                    Adversary1_agent.set_destination(new_dest)
                    new_speed =speed_array[dest_index] 
                    Adversary1_agent.set_target_speed(new_speed)
                                        
                else:
                    #print("The target has been reached, stopping the simulation")
                    break
            
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

    args = argparser.parse_args()

    

    log_level = logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    #logging.info('listening to server %s:%s', args.host, args.port)

    #print(__doc__)

    
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