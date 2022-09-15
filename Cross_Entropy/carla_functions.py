#!/usr/bin/env python

from abc import abstractmethod
import sys
import glob
import os
import math
from shapely.geometry import Polygon
from shapely import affinity

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

class CarlaScenario(object):
    def __init__(self):
        self.world = None
        self.ego = None
        self.adv = None
        self.score = None
        self.purpose = None
        self.feature_vector = []

    def execute_scenario(self, args, parameters):
        bounding_boxes = None
        flag = 0

        adversary_target_speed = parameters[0]
        try:
            client = carla.Client(args.host,args.port)
            client.set_timeout(4.0)

            self.world = client.get_world()
            world = self.world

            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            if(args.no_render): settings.no_rendering_mode = True
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
            self.ego = self.world.try_spawn_actor(ego_blueprint,ego_spawn_point)
            self.adv = self.world.try_spawn_actor(adversary_blueprint,adversary_spawn_point)

            for i in range(0,30):
                world.tick()
            
            ego_agent = BasicAgent(self.ego, target_speed = 10,opt_dict={'ignore_traffic_lights':'True','base_vehicle_threshold':20.0})
            ego_destination = spawn_points[31].location
            #draw_location(world, ego_destination)
            ego_agent.set_destination(ego_destination)
            #adv_destination = carla.Location(x=-94.079308, y=24.415840, z=0.031020)
            adv_destination = spawn_points[31].location
            adv_agent = SimpleAgent(self.adv, adv_destination, target_speed = adversary_target_speed * 3.6)

            self.score = CarlaScenario.get_2D_distance(adversary_spawn_point.location,ego_spawn_point.location)

            counter = 0
            while True:
                world.tick()
                world_snapshot = world.get_snapshot()
                if(self.purpose == 'label'):
                    self.get_features(world_snapshot)
                frame = world_snapshot.timestamp.frame
                dist = CarlaScenario.get_2D_distance(self.ego.get_transform().location,self.adv.get_transform().location)
                if(dist<self.score): 
                    self.score=dist
                    bounding_boxes = []
                    for actor_snapshot in world.get_actors().filter('*vehicle*'):
                        #print(actor_snapshot.bounding_box)
                        actual_actor = world.get_actor(actor_snapshot.id)
                        bb = [actual_actor.bounding_box.extent.x,actual_actor.bounding_box.extent.y,actual_actor.bounding_box.extent.z]
                        #world.debug.draw_box(carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(bb[0],bb[1],bb[2])),actor_snapshot.get_transform().rotation,0.1,carla.Color(0,255,0),30)
                        bounding_boxes.append((carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(bb[0],bb[1],bb[2])),actor_snapshot.get_transform()))
                    
                ego_done = ego_agent.done()
                adv_done = adv_agent.done()
                if(ego_done or adv_done): break
                else:
                    self.ego.apply_control(ego_agent.run_step())
                    self.adv.apply_control(adv_agent.run_step())
                counter +=1

        except KeyboardInterrupt:
                flag = -1
                print("Execute_scenario cancelled by user!")         
            
        finally:
            if(not args.no_render):
                if(bounding_boxes is not None):
                    for box in bounding_boxes:
                        world.debug.draw_box(box[0],box[1].rotation,0.1,carla.Color(0,255,0),1)
                    CarlaScenario.bb_test_code(bounding_boxes)
            if world is not None:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
                if self.ego is not None:
                    self.ego.destroy()
                if self.adv is not None:
                    self.adv.destroy()
            return self.score, flag
    
    def get_features(self, world_snapshot):
        frame = world_snapshot.frame
    
    def write_features(self):
        print("Writing features")
        headers = ['frame']

    @abstractmethod
    def bb_test_code(bounding_boxes):
        polygons = []
        for bb in bounding_boxes:
            vertices = bb[0].get_local_vertices()
            coords = []
            for vert in vertices:
                if(vert.z > 0):
                    coords.append((vert.x,vert.y))
            last_elem = coords[-1]
            coords[-2] = coords[-1]
            coords[-1] = last_elem
            p = Polygon(coords)
            polygons.append(p)
        for polygon in polygons:
            rot_p = affinity.rotate(p, 90, origin='centroid')
            print(p.is_valid,p)

                
    @abstractmethod
    def draw_location(world, location, draw_time = 10):
            world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)

    @abstractmethod
    def get_2D_distance(loc1, loc2):
        return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)

    