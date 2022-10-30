#!/usr/bin/env python   

from abc import abstractmethod
import carla
import math

def draw_spawn_point_locations(world, spawn_points):
    for i in range(len(spawn_points)):
        world.debug.draw_string(spawn_points[i].location,f"{i}",life_time = 30)

def draw_location_on_grid(world, location, draw_time = 10):
        world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)

# ==============================================================================
# -- Grid ---------------------------------------------------------------
# ==============================================================================

class Grid(object):
    def __init__(self, top: float, bottom: float, left: float, right: float, draw_time: int = 0):
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
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right
        self.box_width = abs(self.right - self.left)/20
        self.box_height = abs(self.top - self.bottom)/20 

    def return_location_from_grid(self, i: int, j: int, draw_time: int = 0):
        center_point_y = self.left + self.box_width*(j) + self.box_width/2
        center_point_x = self.top - self.box_height*(i) - self.box_height/2
        location = carla.Location(x=center_point_x,y=center_point_y,z=1)
        return location
    
    def return_grid_from_location(self, location):
        i = 0
        j = 0
        for index in range(0,19):
            if(location.x < self.top - self.box_height * (index)): i=index
            if(location.y > self.left + self.box_width* (index)): j=index
        return (i,j)
    
    def return_coords_from_point(self, point):
        i = math.floor(int(point)/20)
        j = int(point) % 20
        return (i,j)
    
    def return_point_from_coords(i, j):
        return i * 20 + j
    
    def draw_location_on_grid(world, location, draw_time = 10):
        world.debug.draw_point(location,size=0.2,color=carla.Color(0,255,0),life_time=draw_time)
            
    def draw_grid(self, world, draw_time = 10):
        #draw vertical lines
        y=self.left
        for i in range(0,21):
            world.debug.draw_line(carla.Location(x=self.top,y=y,z=12), carla.Location(x=self.bottom,y=y,z=12), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            y+=self.box_width
        #draw horizontal lines
        x=self.bottom
        for i in range(0,21):
            world.debug.draw_line(carla.Location(x=x,y=self.left,z=12), carla.Location(x=x,y=self.right,z=12), thickness=0.1, color=carla.Color(255,0,0), life_time=draw_time)
            x+=self.box_height

    def convert_points_to_locations(self, array):
        locations = []
        for point in array:
            i, j = self.return_coords_from_point(point) 
            dest = self.return_location_from_grid(i,j)
            locations.append(dest)
        return locations
    
    def draw_points_and_locations(world, points):
        counter = 0
        for point in points:
            world.debug.draw_point(point.location,size=0.2,color=carla.Color(255,255,255),life_time=30)
            #self.world.debug.draw_string(point.location + carla.Location(z=3),str(point.location.x)+',\n'+str(point.location.y),color=carla.Color(255,0,0),life_time=30)
            world.debug.draw_string(point.location + carla.Location(z=3),str(counter),color=carla.Color(255,0,0),life_time=30)
            counter += 1
    

world = None
try:
    client = carla.Client('localhost', 2004)
    client.set_timeout(10.0)
    
    world = client.get_world()
    
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    draw_spawn_point_locations(world,world.get_map().get_spawn_points())
    grid = Grid(0,-100,22,42)
    print(world.get_map().get_spawn_points()[333])
    grid.draw_grid(world, draw_time = 30)


    #This is necessary to ensure vehicle "stabilizes" after "falling"
    for i in range (0,30):
        world.tick()
    

finally:
    if world is not None:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
