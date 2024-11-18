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

class Monitor(object):
    def __init__(self, knowledge, vehicle):
        self.vehicle = vehicle
        self.knowledge = knowledge
        weak_self = weakref.ref(self)

        self.knowledge.update_data("location", self.vehicle.get_transform().location)
        self.knowledge.update_data("rotation", self.vehicle.get_transform().rotation)

        world = self.vehicle.get_world()
        bp = world.get_blueprint_library().find("sensor.other.lane_invasion")
        self.lane_detector = world.spawn_actor(
            bp, carla.Transform(), attach_to=self.vehicle
        )
        self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))
        # Adding a Collision Detector
        self.collision_sensor = world.spawn_actor(
            world.get_blueprint_library().find("sensor.other.collision"),
            carla.Transform(),
            attach_to=self.vehicle,
        )
        self.collision_sensor.listen(
            lambda event: self.knowledge.update_status(data.Status.CRASHED)
        )

        # create LIDAR sensor
        self.setup_lidar(world)

        # create depth sensor

    # convert lidar information into an np array and send it to knowledge
    def lidar_callback(self, point_cloud):
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype("f4")))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))
        self.knowledge.update_data("lidar_data", data)

    def setup_lidar(self, world):

        # setup lidar blueprints and attributes
        lidar_bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        lidar_bp.set_attribute("range", str(30))
        lidar_bp.set_attribute("noise_stddev", str(0.1))
        lidar_bp.set_attribute("upper_fov", str(15.0))
        lidar_bp.set_attribute("lower_fov", str(-25.0))
        lidar_bp.set_attribute("channels", str(64.0))
        lidar_bp.set_attribute("points_per_second", str(50000))
        lidar_bp.set_attribute("rotation_frequency", str(20.0))
        lidar_transform = carla.Transform(carla.Location(z=2))

        # create lidar sensor
        self.lidar_sensor = world.spawn_actor(
            lidar_bp, lidar_transform, attach_to=self.vehicle
        )
        self.lidar_sensor.listen(self.lidar_callback)

    def check_vehicle_traffic_light(self):
        if self.vehicle.is_at_traffic_light():
            traffic_light = self.vehicle.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                return True
        return False

    # Function that is called at time intervals to update ai-state
    def update(self, time_elapsed):
        # Update the position of vehicle into knowledge
        self.knowledge.update_data("location", self.vehicle.get_transform().location)
        self.knowledge.update_data("rotation", self.vehicle.get_transform().rotation)

        #update the traffic state
        self.knowledge.update_data("is_at_traffic_light", self.check_vehicle_traffic_light())

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.knowledge.update_data("lane_invasion", event.crossed_lane_markings)
