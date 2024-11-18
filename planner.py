import glob
import os
import sys
from collections import deque
import math
import numpy as np

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

import carla
import knowledge as data
from knowledge import Status

class Planner(object):
    def __init__(self, knowledge, vehicle):
        self.knowledge = knowledge
        self.vehicle = vehicle
        self.path = deque([])

    # Create a map of waypoints to follow to the destination and save it
    def make_plan(self, source, destination):
        self.path = self.build_path(source, destination)
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        self.update_plan()
        self.knowledge.update_destination(self.get_current_destination())
        obstacles = self.knowledge.get_obstacles()
        if obstacles is None:
            obstacles = []

    # Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
    def update_plan(self):
        if len(self.path) == 0:
            return

        if self.knowledge.arrived_at(self.path[0]):
            self.path.popleft()

        if len(self.path) == 0:
            self.knowledge.update_status(Status.ARRIVED)

    def is_space_available(self, location):
        # Implement logic to check if the location is free from obstacles
        for obstacle in self.knowledge.get_obstacles():
            if location.distance(obstacle) < 3.0:  # Adjust the distance threshold
                return False
        return True

    def calculate_detour(self, vehicle_location, obstacle_location):

        DETOUR_THRESHOLD = 0.8
        # Calculate the direction vector from vehicle to obstacle
        direction_to_obstacle = obstacle_location - vehicle_location
        distance_to_obstacle = direction_to_obstacle.length()

        # Normalize the direction vector
        direction_to_obstacle /= distance_to_obstacle

        # Perpendicular vectors for left and right directions
        left_direction = carla.Location(
            -direction_to_obstacle.y, direction_to_obstacle.x, 0
        )
        right_direction = carla.Location(
            direction_to_obstacle.y, -direction_to_obstacle.x, 0
        )

        # Check space on the left
        left_detour = (
            vehicle_location + left_direction * DETOUR_THRESHOLD
        )  # Adjust the detour distance
        if self.is_space_available(left_detour):
            return left_detour

        # Check space on the right
        right_detour = (
            vehicle_location + right_direction * DETOUR_THRESHOLD
        )  # Adjust the detour distance
        if self.is_space_available(right_detour):
            return right_detour

        # If obstacle is directly in front, try going around it
        front_left_detour = (
            vehicle_location
            + direction_to_obstacle * DETOUR_THRESHOLD
            + left_direction * DETOUR_THRESHOLD
        )
        if self.is_space_available(front_left_detour):
            return front_left_detour

        front_right_detour = (
            vehicle_location
            + direction_to_obstacle * DETOUR_THRESHOLD
            + right_direction * DETOUR_THRESHOLD
        )
        if self.is_space_available(front_right_detour):
            return front_right_detour

        # If no detour is possible, return None
        return None

    # get current destination
    def get_current_destination(self):
        status = self.knowledge.get_status()
        # if we are driving, then the current destination is next waypoint
        if status == Status.DRIVING:
            # n_distance = self.path[0].distance(self.knowledge.get_location())
            # print("Distance To: ", n_distance)
            # TODO: Take into account traffic lights and other cars
            self.knowledge.update_data("target_speed", 8)
            if self.path is None or len(self.path) == 0:
                return self.knowledge.get_location()
            return self.path[0]
        if status == Status.ARRIVED:
            self.knowledge.update_data("target_speed", 0)
            return self.knowledge.get_location()
        if status == Status.REDLIGHT:
            self.knowledge.update_data("target_speed", 0)
            return self.knowledge.get_location()
        if status == Status.HEALING:
            self.knowledge.update_data("target_speed", 0.5)
            # Add new destinations if new obstacles are detected
            obstacles = self.knowledge.get_obstacles()
            for obstacle_location in obstacles:
                vehicle_location = self.knowledge.get_location()
                # print(obstacle)
                if (
                    vehicle_location.distance(obstacle_location) < 3.0
                ):  # Check for nearby obstacles
                    detour_destination = self.calculate_detour(
                        vehicle_location, obstacle_location
                    )
                    if detour_destination:
                        self.knowledge.update_data("target_speed", 0.5)
                        self.path.appendleft(detour_destination)
                        print("Taking DETOUR")

                        world = self.vehicle.get_world()
                        world.debug.draw_string(
                            detour_destination,
                            "^",
                            draw_shadow=True,
                            color=carla.Color(r=255, g=0, b=0),
                            life_time=600.0,
                            persistent_lines=True,
                        )
                        break

                    else:
                        self.knowledge.update_data("target_speed", 0.0)
                        print("Stopping Due to Healing")
                        return self.knowledge.get_location()

            # TODO: Implement crash handling. Probably needs to be done by following waypoint list to exit the crash site.
            # Afterwards needs to remake the path.
            # self.knowledge.update_status(Status.DRIVING
            if self.path is None or len(self.path) == 0:
                return self.knowledge.get_location()
            return self.path[0]
        if status == Status.CRASHED:
            # TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state.
            # You should use separate waypoint list for that, to not mess with the original path.
            return self.knowledge.get_location()
        # otherwise destination is same as current position
        return self.knowledge.get_location()

    # TODO: Implementation
    # TODO: create path of waypoints from source to destination

    def build_path(self, source, destination):
        self.path = deque([])

        world = self.vehicle.get_world()
        world_map = world.get_map()

        # Get Waypoints from source to destination using Carla's map API
        source_waypoint = world_map.get_waypoint(source.location)
        destination_waypoint = world_map.get_waypoint(destination)

        # Generating Waypoints with less than 5 meters interval
        current_waypoint = source_waypoint
        count = 0
        PATH_THRESHOLD = source.location.distance(destination) / 5 + 10

        while current_waypoint.transform.location.distance(destination) > 5.01:
            next_waypoints = current_waypoint.next(5.0)

            if len(next_waypoints) == 0:
                break

            next_waypoint = min(
                next_waypoints,
                key=lambda wp: wp.transform.location.distance(destination),
            )

            '''
            # Determine if a lane change is necessary based on the destination's relative position
            destination_direction = (
                destination_waypoint.transform.location
                - current_waypoint.transform.location
            )
            destination_direction /= (
                destination_direction.length()
            )  # Normalize the vector

            current_forward_vector = current_waypoint.transform.get_forward_vector()
            cross_product = (
                current_forward_vector.x * destination_direction.y
                - current_forward_vector.y * destination_direction.x
            )

            # If destination is to the right, consider changing to the right lane
            if cross_product > 0:
                possible_waypoint = current_waypoint.get_right_lane()
                if (
                    possible_waypoint
                    and possible_waypoint.lane_type == carla.LaneType.Driving
                ):
                    next_waypoint = possible_waypoint

            # If destination is to the left, consider changing to the left lane
            elif cross_product < 0:
                possible_waypoint = current_waypoint.get_left_lane()
                if (
                    possible_waypoint
                    and possible_waypoint.lane_type == carla.LaneType.Driving
                ):
                    next_waypoint = possible_waypoint'''

            self.path.append(next_waypoint.transform.location)
            world.debug.draw_string(
                next_waypoint.transform.location,
                "^",
                draw_shadow=True,
                color=carla.Color(r=255, g=0, b=0),
                life_time=600.0,
                persistent_lines=True,
            )

            current_waypoint = next_waypoint
            count += 1
            if count > PATH_THRESHOLD:
                break

        self.path.append(destination)
        return self.path
