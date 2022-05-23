#!/usr/bin/env python

# Author:           Matthew Litton
# Last Modified:    5/23/2022
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

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
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

    def __init__(self, carla_world, grid, args):#hud, args):
        """Constructor method"""
        self._grid = grid
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        #self.hud = hud
        self.player = None
        self.Adversary_1 = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self.restart(args)
        #self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self, args):
        """Restart the world"""
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_id = self.camera_manager.transform_index if self.camera_manager is not None else 0

        # Get a random blueprint.
        """
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        """
        blueprints = self.world.get_blueprint_library()
        blueprint=blueprints.filter("diamondback")[0]
        blueprint.set_attribute('role_name', 'hero')

        Adversary1_blueprint = blueprints.filter("vehicle.dodge.charger_police")[0]
        Adversary1_blueprint.set_attribute('role_name','Adversary1')
        print(Adversary1_blueprint)

        # Spawn the player.
        """
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        """
        spawn_point = carla.Transform(carla.Location(self._grid.return_location_from_grid(19,19)),carla.Rotation(roll=0,pitch=0,yaw=0))
        self.player = self.world.try_spawn_actor(blueprint,spawn_point)
        self.modify_vehicle_physics(self.player)
        

        A1_spawn_point = carla.Transform(carla.Location(self._grid.return_location_from_grid(7,20)),carla.Rotation(roll=0,pitch=0,yaw=-90))
        self.Adversary_1 = self.world.try_spawn_actor(Adversary1_blueprint,A1_spawn_point)
        print(self.Adversary_1)
        self.modify_vehicle_physics(self.Adversary_1)
        
        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

        # Set up the sensors.
        """
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_id
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)
        """
    def next_weather(self, reverse=False):
        """Get next weather setting"""
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        """Method for every tick"""
        self.hud.tick(self, clock)

    def render(self, display):
        """Render world"""
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy_sensors(self):
        """Destroy sensors"""
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        """Destroys all actors"""
        actors = [
            #self.camera_manager.sensor,
            #self.collision_sensor.sensor,
            #self.lane_invasion_sensor.sensor,
            #self.gnss_sensor.sensor,
            self.player,self.Adversary_1]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- Grid ---------------------------------------------------------------
# ==============================================================================

class Grid(object):
    def __init__(self, world, top: float, bottom: float, left: float, right: float, draw_time: int = 10):
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
        Grid.draw_grid(self, draw_time)

    

    def return_location_from_grid(self, i: int, j: int, draw: bool = False):
        center_point_y = self.left + self.box_width*(j-1) + self.box_width/2
        #print(center_point_y)
        center_point_x = self.top - self.box_height*(i-1) - self.box_height/2
        #print(center_point_x)
        location = carla.Location(x=center_point_x,y=center_point_y,z=1)
        #print(location)
        if(draw): self.draw_location_on_grid(location)
        return location

    def draw_location_on_grid(self, location, draw_time = 10):
        #print("Drawing point")
        #self.world.debug.draw_string(location, 'OOO', draw_shadow = False, color=carla.Color(0,255,0), life_time=10)
        self.world.debug.draw_point(location,size=0.5,color=carla.Color(0,255,0),life_time=draw_time)
            
    def draw_grid(self, draw_time):
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
    Main loop of the simulation. It handles updating all the HUD information,
    ticking the agent and, if needed, the world.
    """

    #pygame.init()
    #pygame.font.init()
    world = None

    try:
        if args.seed:
            random.seed(args.seed)

        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        traffic_manager = client.get_trafficmanager()
        sim_world = client.get_world()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager.set_synchronous_mode(True)

        #display = pygame.display.set_mode(
        #    (args.width, args.height),
        #    pygame.HWSURFACE | pygame.DOUBLEBUF)

        #hud = HUD(args.width, args.height)
        grid = Grid(client.get_world(),-33,-61,4,38, draw_time = 20)
        loc = grid.return_location_from_grid(4,1,draw=True)
        world = World(client.get_world(), grid, args)#hud, args)
        spectator = world.world.get_spectator()
        
        #middle of the grid
        spectator.set_transform(carla.Transform(carla.Location(x=-47,y=21,z=20),carla.Rotation(roll=0, pitch=-90,yaw=0)))
        #controller = KeyboardControl(world)

        #if args.agent == "Basic":
        #    agent = BasicAgent(world.player)
        #else:
        #    agent = BehaviorAgent(world.player, behavior=args.behavior)
        
        # Set the agent destination
        #spawn_points = world.map.get_spawn_points()
        #destination = random.choice(spawn_points).location
        d1 = grid.return_location_from_grid(20,20)
        d2 = grid.return_location_from_grid(20,1)
        d3 = grid.return_location_from_grid(1,20)
        d4 = grid.return_location_from_grid(1,1)
        destination_array=[d1,d2,d3,d4]
        #destination = random.choice(destination_array)
        destination = grid.return_location_from_grid(4,1, draw=True)
        agent = SimpleAgent(world.player, destination, target_speed=80)
        
        
        agent2 = BasicAgent(world.Adversary_1)
        agent2_dest = grid.return_location_from_grid(5,0)
        waypoint = world.world.get_map().get_waypoint(agent2_dest,project_to_road=True, lane_type=(carla.LaneType.Driving))
        print(waypoint)
        
        
        
        #agent2.set_destination(waypoint)
        
       

        #clock = pygame.time.Clock()
        
        while True:
            #clock.tick()
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()
            #if controller.parse_events():
            #    return

            #world.tick(clock)
            #world.render(display)
            #pygame.display.flip()
            vel = world.player.get_velocity()
            speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            print(f"Bike/Player speed: {speed}")
            
        
            if agent.done():
                if args.loop:
                    print("The target has been reached, searching for another target")
                    d = random.choice(destination_array)
                    agent.set_destination(d)
                    agent.set_target_speed(60)
                    print(f"Setting target speed to {d.x}")
                    #world.hud.notification("The target has been reached, searching for another target", seconds=4.0)
                    
                else:
                    print("The target has been reached, stopping the simulation")
                    break

            control = agent.run_step()
            control.manual_gear_shift = False
            #spectator.set_transform(carla.Transform(world.player.get_transform().location+carla.Location(z=3),world.player.get_transform().rotation))
            world.player.apply_control(control)

            world.Adversary_1.apply_control(agent2.run_step())

            

    finally:

        if world is not None:
            settings = world.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)
            traffic_manager.set_synchronous_mode(True)

            world.destroy()

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================


def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
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
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()