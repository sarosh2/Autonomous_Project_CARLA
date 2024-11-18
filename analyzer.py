import glob
import os
import sys

try:
    sys.path.append(
        glob.glob(
            "**/*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import weakref
import carla
import knowledge as data
import numpy as np

class Analyser(object):
    def __init__(self, knowledge, vehicle):
        self.knowledge = knowledge
        self.vehicle = vehicle
        self.is_lidar_below_threshold = False
        self.obstacle_threshold = 1.0
        self.vehicle_threshold = 10.0

    def detect_obstacle(self, data):
        distance = self.get_distance(data)
        if distance < self.obstacle_threshold:  # Example threshold for obstacles
            # print('Obstacle detected. : ', data)
            obstacle_location = carla.Location(float(data[0]), float(data[1]), float(data[2]))
            return obstacle_location
        else:
            return None
    
    def get_distance(self, pdata):
        return np.sqrt(pdata[0] ** 2 + pdata[1] ** 2)

    def is_vehicle_obstacle(self, pdata):
        world = self.vehicle.get_world()
        obstacle_location = carla.Location(float(pdata[0]), float(pdata[1]), float(pdata[2]))
        vehicles = world.get_actors().filter("vehicle.*")

        for vehicle in vehicles:
            if vehicle.id != self.vehicle.id:
              vehicle_location = vehicle.get_transform().location
              distance = obstacle_location.distance(vehicle_location)
              if distance < 2.0:
                  return True
        return False

    def analyse_lidar(self):
        lidar_data = self.knowledge.get_lidar_data()

        if lidar_data is None:
            print("Lidar data is None")
            return

        obstacles = []
        is_vehicle = False

        for pdata in lidar_data:
          
            if self.get_distance(pdata) < self.vehicle_threshold:
                #clear
                #print(" Checking for vehicle")
                if self.is_vehicle_obstacle(pdata):
                  is_vehicle = True
                  print("Vehicle detected")
                  obstacles.append(carla.Location(float(pdata[0]), float(pdata[1]), float(pdata[2])))
                  break
            obstacle = self.detect_obstacle(pdata)
            if obstacle is not None:
                obstacles.append(obstacle)

        if len(obstacles) == 0:
            self.knowledge.update_status(data.Status.DRIVING)
        else:
            self.knowledge.update_data("is_vehicle", is_vehicle)
            self.knowledge.update_data("obstacles", obstacles)
            self.knowledge.update_status(data.Status.HEALING)

    def analyse_obstacles(self):
        obstacles = self.knowledge.get_obstacles()
        if obstacles is None:
            return
        for obstacle in obstacles:
            world = self.vehicle.get_world()
            '''
            world.debug.draw_string(
                obstacle,
                "O",
                draw_shadow=False,
                color=carla.Color(r=255, g=0, b=0),
                life_time=0.5,
                persistent_lines=True,
            )'''
    # Analyse Traffic Light
    def analyse_traffic_light(self):
        return self.knowledge.get_traffic_light_state()

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        if self.knowledge.get_status() == data.Status.CRASHED:
            return
        if self.analyse_traffic_light():
            self.knowledge.update_status(data.Status.REDLIGHT)
            return
        self.analyse_lidar()
        self.analyse_obstacles()
        print("Lidar Data from Knowledge: ", self.knowledge.get_status())
        return
