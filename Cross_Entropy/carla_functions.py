#!/usr/bin/env python

from abc import abstractmethod
import sys
import glob
import os
import math
import numpy as np
import pandas as pd
import time
import csv
from shapely.geometry import Polygon
from shapely.affinity import rotate

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.simple_agent import SimpleAgent
from examples.Execute_scenario import read_csv

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
        self.feature_vector = []
        self.ego_start = 60
        self.ego_dest = 8
        self.ego_speed = 10
        self.adv_start = 13
        self.adv_dest = 58
        self.adv_speed = None

    def pre_score(self, parameters):
        bad_path = False
        adversary_target_speed = parameters[0]
        if(adversary_target_speed < 1):
            bad_path = True
            return (0, bad_path)
        return(self.score,bad_path)

    def execute_scenario(self, args, parameters, purpose, file=None):
        bounding_boxes = None
        bounding_boxes_draw = None
        bounding_boxes_closest = None
        flag = 0

        #should only be in effect for GOOD search
        """
        pre_score_ret = self.pre_score(parameters)
        if(pre_score_ret[1]):
            return (pre_score_ret[0],flag)
        """

        if(purpose == "replay" and file is not None):
            self.assign_parameters(file)

        adversary_target_speed = parameters[0]
        if self.adv_speed is None:
            self.adv_speed = adversary_target_speed
        try:
            client = carla.Client(args.host,args.port)
            client.set_timeout(4.0)

            self.world = client.get_world()
            world = self.world

            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            if(args.no_render): settings.no_rendering_mode = True
            else: settings.no_rendering_mode = False
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
            #self.draw_points_and_locations(spawn_points)
            ego_spawn_point = spawn_points[self.ego_start]
            adversary_spawn_point = spawn_points[self.adv_start]
            #adversary_spawn_point = carla.Transform(carla.Location(x=-67.668266, y=5.793464, z=0.275),carla.Rotation(roll=0,pitch=0,yaw=-160))
            #adversary_spawn_point = carla.Transform(carla.Location(x=-82.668266, y=8.793464, z=0.275),carla.Rotation(roll=0,pitch=0,yaw=-160))
            #draw_location(world, ego_spawn_point.location)
            self.ego = self.world.try_spawn_actor(ego_blueprint,ego_spawn_point)

            adversary_sp_mod = carla.Transform(carla.Location(adversary_spawn_point.location-carla.Location(x=10)),adversary_spawn_point.rotation)
            self.adv = self.world.try_spawn_actor(adversary_blueprint,adversary_spawn_point)

            for i in range(0,30):
                world.tick()
            
            ego_agent = BasicAgent(self.ego, target_speed = self.ego_speed,opt_dict={'ignore_traffic_lights':'True','base_vehicle_threshold':20.0})
            ego_destination = spawn_points[self.ego_dest].location
            #draw_location(world, ego_destination)
            ego_agent.set_destination(ego_destination)
            #adv_destination = carla.Location(x=-94.079308, y=24.415840, z=0.031020)
            adv_destination = spawn_points[self.adv_dest].location
            adv_agent = SimpleAgent(self.adv, adv_destination, target_speed = self.adv_speed * 3.6)

            self.score = CarlaScenario.get_2D_distance(adversary_spawn_point.location,ego_spawn_point.location)

            counter = 0
            stuck_counter = 0
            while True:
                world.tick()
                stuck_counter += 1

                if(stuck_counter % 200 == 0):
                    if(CarlaScenario.get_vehicle_speed(self.adv) < 0.1):
                        print(f"Exiting because stuck: {self.adv_speed}")
                        break
                

                actor_snapshots = world.get_actors().filter('*vehicle*')
                bounding_boxes = []
                for actor_snapshot in actor_snapshots:
                    actual_actor = world.get_actor(actor_snapshot.id)
                    bb = [actual_actor.bounding_box.extent.x,actual_actor.bounding_box.extent.y,actual_actor.bounding_box.extent.z]
                    bounding_boxes.append((carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(y=bb[0],x=bb[1],z=bb[2])),actor_snapshot.get_transform(),actual_actor.attributes.get('role_name')))

                bb_ret = CarlaScenario.bb_test_code(bounding_boxes)

                if(bb_ret[1] < self.score):
                    self.score = bb_ret[1]
                    bounding_boxes_draw = []
                    bounding_boxes_closest = []
                    for actor_snapshot in actor_snapshots:
                        actual_actor = world.get_actor(actor_snapshot.id)
                        bb = [actual_actor.bounding_box.extent.x,actual_actor.bounding_box.extent.y,actual_actor.bounding_box.extent.z]
                        bounding_boxes_draw.append((carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(x=bb[0],y=bb[1],z=bb[2])),actor_snapshot.get_transform(),actual_actor.attributes.get('role_name')))
                        bounding_boxes_closest.append((carla.BoundingBox(actor_snapshot.get_transform().location,carla.Vector3D(y=bb[0],x=bb[1],z=bb[2])),actor_snapshot.get_transform(),actual_actor.attributes.get('role_name')))
                
                world_snapshot = world.get_snapshot()
                if(purpose == "label"):
                    self.get_features(world_snapshot,bb_ret)
                frame = world_snapshot.timestamp.frame

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
                if(bounding_boxes_draw is not None):
                    for box in bounding_boxes_draw:
                        world.debug.draw_box(box[0],box[1].rotation,0.1,carla.Color(0,255,0),1)
            if(bounding_boxes_closest is not None):
                CarlaScenario.bb_final_code(bounding_boxes_closest)
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
    
    def assign_parameters(self, file):
        df = pd.read_csv(file)
        self.ego_start = df['ego_start'][0]
        self.ego_dest = df['ego_dest'][0]
        self.ego_speed = df['ego_speed'][0]
        self.adv_start = df['adv_start'][0]
        self.adv_dest = df['adv_dest'][0]
        self.adv_speed = df['adv_vel'][0]

    def draw_points_and_locations(self, points):
        counter = 0
        for point in points:
            self.world.debug.draw_point(point.location,size=0.2,color=carla.Color(255,255,255),life_time=30)
            #self.world.debug.draw_string(point.location + carla.Location(z=3),str(point.location.x)+',\n'+str(point.location.y),color=carla.Color(255,0,0),life_time=30)
            self.world.debug.draw_string(point.location + carla.Location(z=3),str(counter),color=carla.Color(255,0,0),life_time=30)
            counter += 1
    
    def get_features(self, world_snapshot, bb_ret):
        frame_feature_vec = []
        frame_feature_vec.append(world_snapshot.frame)
        frame_feature_vec.append(bb_ret[0])
        frame_feature_vec.append(bb_ret[1])
        frame_feature_vec.append(bb_ret[2])

        actors = []
        ego_snapshot = world_snapshot.find(self.ego.id)
        adv_snapshot = world_snapshot.find(self.adv.id)
        actors.append(ego_snapshot)
        actors.append(adv_snapshot)

        for a in actors:
            vel = a.get_velocity() #m/s
            accel = a.get_acceleration() #m/s^2
            ang_vel = a.get_angular_velocity() #deg/s
            frame_feature_vec.append(vel.x)
            frame_feature_vec.append(vel.y)
            frame_feature_vec.append(vel.z)
            frame_feature_vec.append(accel.x)
            frame_feature_vec.append(accel.y)
            frame_feature_vec.append(accel.z)
            frame_feature_vec.append(ang_vel.x)
            frame_feature_vec.append(ang_vel.y)
            frame_feature_vec.append(ang_vel.z)
        self.feature_vector.append(frame_feature_vec)
        
    
    def write_features(self):
        print("Writing features")
        headers = ['frame', 'intersect','hausdorff_distance','angle', 
                   'ego_vel_x','ego_vel_y','ego_vel_z','ego_accel_x','ego_accel_y','ego_accel_z','ego_ang_vel_x','ego_ang_vel_y','ego_ang_vel_z',
                   'adv_vel_x','adv_vel_y','adv_vel_z','adv_accel_x','adv_accel_y','adv_accel_z','adv_ang_vel_x','adv_ang_vel_y','adv_ang_vel_z']
        df = pd.DataFrame(data = self.feature_vector, columns=headers)
        data_path = "c:\\data\\label\\"
        time_str = time.strftime("%Y%m%d-%H%M%S")
        file_name = (time_str + "_" + str(round(self.score)) +"_0.csv")
        full_path = os.path.join(data_path,file_name)
        df.to_csv(full_path)

        path_file_name = file_name.split('.csv')[0] + "_path.csv"
        with open(os.path.join(data_path,path_file_name),'w') as f:
            writer = csv.writer(f)
            writer.writerow(['ego_start','ego_dest','ego_speed','adv_start','adv_dest','adv_vel'])
            writer.writerow([self.ego_start, self.ego_dest,self.ego_speed,self.adv_start,self.adv_dest,self.adv_speed])

    @abstractmethod
    def bb_test_code(bounding_boxes):
        polygons = {}
        transforms = {}
        for bb in bounding_boxes:
            vertices = bb[0].get_local_vertices()
            coords = []
            for vert in vertices:
                if(vert.z > 0):
                    coords.append((vert.x,vert.y))
            coords_copy = coords[:]
            coords[-2] = coords_copy[-1]
            coords[-1] = coords_copy[-2]
            p = Polygon(coords)
            carla_yaw = bb[1].rotation.yaw
            if(carla_yaw > 0):
                p = rotate(p,carla_yaw - 90)
            elif(carla_yaw < 0):
                p = rotate(p,abs(carla_yaw) + 90)
            polygons[bb[2]] = p
            transforms[bb[2]] = bb[1]
            #print(bb[2],list(p.exterior.coords))
        
        ego_vec = (transforms['ego'].get_forward_vector().x,transforms['ego'].get_forward_vector().y)
        diff_vec = transforms['adversary'].location - transforms['ego'].location 
        angle = CarlaScenario.angle_between(ego_vec,(diff_vec.x,diff_vec.y))
        dist = polygons['ego'].distance(polygons['adversary'])
        accident = polygons['ego'].intersects(polygons['adversary'])
        return (accident, dist, angle)

    @abstractmethod
    def bb_final_code(bounding_boxes):
        polygons = {}
        transforms = {}
        for bb in bounding_boxes:
            vertices = bb[0].get_local_vertices()
            coords = []
            for vert in vertices:
                if(vert.z > 0):
                    coords.append((vert.x,vert.y))
            coords_copy = coords[:]
            coords[-2] = coords_copy[-1]
            coords[-1] = coords_copy[-2]
            p = Polygon(coords)
            #print(bb[2],list(p.exterior.coords))
            carla_yaw = bb[1].rotation.yaw
            if(carla_yaw > 0):
                p = rotate(p,carla_yaw - 90)
            elif(carla_yaw < 0):
                #p = rotate(p,abs(carla_yaw) + 90)
                p = rotate(p,(180+carla_yaw) + 90)
            polygons[bb[2]] = p
            transforms[bb[2]] = bb[1]
            print(bb[2],list(p.exterior.coords), carla_yaw)
        
        ego_vec = (transforms['ego'].get_forward_vector().x,transforms['ego'].get_forward_vector().y)
        diff_vec = transforms['adversary'].location - transforms['ego'].location 
        angle = CarlaScenario.angle_between(ego_vec,(diff_vec.x,diff_vec.y))
        dist = polygons['ego'].distance(polygons['adversary'])
        accident = polygons['ego'].intersects(polygons['adversary'])
        print(ego_vec, diff_vec, angle, dist, accident)
        
    @abstractmethod
    #pass the ego vector first
    def angle_between(v1,v2):
        v1_u = v1/np.linalg.norm(v1)
        v2_u = v2/np.linalg.norm(v2)
        angle =  math.degrees(math.atan2(v1_u[0],v1_u[1]) - math.atan2(v2_u[0],v2_u[1]))
        if(angle < 0 and abs(angle) > 180): angle +=360
        if(angle > 180): angle = angle-360
        return angle
                
    @abstractmethod
    def draw_location(world, location, draw_time = 10):
            world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)

    @abstractmethod
    def get_2D_distance(loc1, loc2):
        return math.sqrt((loc1.x - loc2.x)**2+(loc1.y-loc2.y)**2)

    @abstractmethod
    def get_vehicle_speed(actor):
        #Returns vehicle speed in km/hr
        vel = actor.get_velocity()
        speed = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)#m/s * 1km/1000m * 3600s/1hr = km/hr (i.e., m/s * 3.6 = km/hr, 3.6 = (km*s)/(m*hr))
        return speed