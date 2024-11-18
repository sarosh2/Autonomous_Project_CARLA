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
from knowledge import Status

class Executor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        self.target_pos = knowledge.get_location()

    # Update the executor at some intervals to steer the car in desired direction
    def update(self, time_elapsed):
        status = self.knowledge.get_status()
        # TODO: this needs to be able to handle
        if status == Status.DRIVING or status == Status.HEALING:
            dest = self.knowledge.get_current_destination()
            self.update_control(dest, [1], time_elapsed)
        if status == Status.CRASHED:
            self.handle_crash()

        if status == Status.REDLIGHT:
            self.handle_redlight()

    def handle_redlight(self):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = False
        self.vehicle.apply_control(control)

    def handle_crash(self):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 1.0
        control.hand_brake = False
        self.vehicle.apply_control(control)

    def calculate_throttle_from_speed(self):
        target_speed = self.knowledge.get_target_speed()
        current_speed = self.vehicle.get_velocity().length()

        # Calculate throttle based on speed difference
        throttle = 0.4
        #print("Current Speed: ", current_speed)
        if current_speed < target_speed:
            throttle = 1.1 * (target_speed - current_speed) / target_speed
        elif current_speed > target_speed:
            throttle = 1.0 * (target_speed - current_speed) / current_speed
        return throttle

    # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
    # TODO: Take into account that exiting the crash site could also be done in reverse, so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
    def update_control(self, destination, additional_vars, delta_time):
        self.vehicle.get_world().debug.draw_string(
            destination,
            "*",
            draw_shadow=True,
            color=carla.Color(r=0, g=255, b=0),
            life_time=600.0,
            persistent_lines=True,
        )

        # Get vehicle's current transform and location
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation

        # Convert vehicle's current location and destination into numpy arrays
        vehicle_pos = np.array([vehicle_location.x, vehicle_location.y])
        destination_pos = np.array([destination.x, destination.y])

        # Calculate the vector from the vehicle to the destination
        vector_to_destination = destination_pos - vehicle_pos
        vector_to_destination_normalized = vector_to_destination / np.linalg.norm(
            vector_to_destination
        )

        # Get vehicle's forward vector
        forward_vector = np.array(
            [
                np.cos(np.radians(vehicle_rotation.yaw)),
                np.sin(np.radians(vehicle_rotation.yaw)),
            ]
        )

        # Dot product and cross product to find the angle to the destination
        dot_product = np.dot(forward_vector, vector_to_destination_normalized)
        cross_product = np.cross(forward_vector, vector_to_destination_normalized)

        # Calculate steering angle (angle between vehicle's forward direction and destination direction)
        angle_to_destination = np.arccos(np.clip(dot_product, -1.0, 1.0))
        steer_direction = np.sign(cross_product)

        # Create vehicle control object
        control = carla.VehicleControl()
        throttle = self.calculate_throttle_from_speed()

        if throttle > 0.0:
            control.throttle = throttle
            control.brake = 0.0

        else:
            control.throttle = 0.0
            control.brake = abs(throttle)

        control.steer = steer_direction * (
            angle_to_destination / np.pi
        )  # Normalize steering angle to [-1, 1]
        control.hand_brake = False

        # Apply the control to the vehicle
        self.vehicle.apply_control(control)