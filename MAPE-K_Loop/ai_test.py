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

import carla
import pygame

import random
import time
import argparse

import custom_ai as ai


def get_dist(point1, point2):
    return point1.location.distance(point2)


def get_start_point(world, coord):
    points = world.get_map().get_spawn_points()
    index = 0
    ti = -1
    td = get_dist(points[0], coord)
    for point in points:
        ti += 1
        d = get_dist(point, coord)
        if d < td:
            td = d
            index = ti
    start_point = points[index]
    return world.get_map().get_waypoint(start_point.location)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        "-m",
        "--milestone-number",
        metavar="M",
        default=1,
        type=int,
        help="Milestone number (default: 1)",
    )
    args = argparser.parse_args()

    actor_list = []

    try:
        # Get CARLA server host from environment variable
        carla_host = os.getenv("CARLA_SERVER_HOST", "localhost")

        # Connect to CARLA
        client = carla.Client(carla_host, 2000)
        client.set_timeout(100.0)
        world = client.get_world()
        world = client.load_world("Town03")
        blueprints = world.get_blueprint_library().filter("vehicle.*")
        blueprints = [
            x for x in blueprints if int(x.get_attribute("number_of_wheels")) == 4
        ]
        blueprints = [x for x in blueprints if not x.id.endswith("isetta")]

        def try_spawn_random_vehicle_at(transform, vid=""):
            if vid != "":
                blueprint = None
                for bp in blueprints:
                    print("{} - {}".format(bp.id, bp.id == vid))
                    if bp.id == vid:
                        blueprint = bp
                        break
                if blueprint.has_attribute("color"):
                    color = random.choice(
                        blueprint.get_attribute("color").recommended_values
                    )
                    blueprint.set_attribute("color", color)
                blueprint.set_attribute("role_name", "autopilot")
                vehicle = world.try_spawn_actor(blueprint, transform)
                if vehicle is not None:
                    actor_list.append(vehicle)
                    print("spawned %r at %s" % (vehicle.type_id, transform.location))
                    return vehicle
            else:
                attempts = 20
                while attempts > 0:
                    blueprint = random.choice(blueprints)
                    if blueprint.has_attribute("color"):
                        color = random.choice(
                            blueprint.get_attribute("color").recommended_values
                        )
                        blueprint.set_attribute("color", color)
                    blueprint.set_attribute("role_name", "autopilot")
                    vehicle = world.try_spawn_actor(blueprint, transform)
                    if vehicle is not None:
                        actor_list.append(vehicle)
                        print(
                            "spawned %r at %s" % (vehicle.type_id, transform.location)
                        )
                        return vehicle
            print("WARNING: vehicle not spawned, NONE returned")
            return None

        # Defining positions
        ex1 = [
            carla.Vector3D(42.5959, -4.3443, 1.8431),
            carla.Vector3D(22, -4, 1.8431),
            carla.Vector3D(9, -22, 1.8431),
        ]
        ex2 = [
            carla.Vector3D(42.5959, -4.3443, 1.8431),
            carla.Vector3D(-30, 167, 1.8431),
        ]

        milestones = [ex1, ex2]
        ms = max(0, min(args.milestone_number - 1, len(milestones) - 1))
        ex = milestones[ms]
        end = ex[len(ex) - 1]
        destination = ex[1]

        # Getting waypoint to spawn
        start = get_start_point(world, ex[0])

        # Spawning
        vehicle = try_spawn_random_vehicle_at(start.transform, "vehicle.nissan.micra")

        if vehicle == None:
            return

        # Setting autopilot
        def route_finished(autopilot):
            pos = autopilot.get_vehicle().get_transform().location
            print("Vehicle arrived at destination: ", pos)
            
            if pos.distance(carla.Location(end)) < 6.0:
                print("Excercise route finished")
                running = False

                # stop the car once the test is done
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.steer = 0.00
                control.brake = 1.0
                control.hand_brake = False
                autopilot.get_vehicle().apply_control(control)
                time.sleep(10000)



            else:
                autopilot.set_destination(end)

        autopilot = ai.Autopilot(vehicle)
        autopilot.set_destination(destination)
        autopilot.set_route_finished_callback(route_finished)

        if ms == 1:
            print("Spawning malicious actor...")
            spawn = start.get_right_lane()
            mal = try_spawn_random_vehicle_at(spawn.transform, "vehicle.nissan.micra")
            bp = world.get_blueprint_library().find("sensor.other.collision")
            sensor = world.spawn_actor(bp, carla.Transform(), attach_to=mal)

            def _on_collision(self, event):
                if not self:
                    return
                print("Collision with: ", event.other_actor.type_id)
                if event.other_actor.type_id.split(".")[0] == "vehicle":
                    print("Test FAILED")
                mal.destroy()
                sensor.destroy()

            sensor.listen(lambda event: _on_collision(mal, event))

            control = carla.VehicleControl()
            control.throttle = 1.0
            control.steer = -0.07
            control.brake = 0.0
            control.hand_brake = False
            mal.apply_control(control)

        ctr = 0
        running = True
        while running:
            status = autopilot.update()
            if status == None:
                ctr += 1
                if ctr > 3:
                    running = False
            else:
                ctr = 0
        time.sleep(2)

    finally:

        print("destroying actors")
        for actor in actor_list:
            actor.destroy()
        print("done.")

        pygame.quit()


if __name__ == "__main__":

    main()
