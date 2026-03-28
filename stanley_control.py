import carla
import random
import time
import math


client = carla.Client("localhost", 2000)
client.set_timeout(5.0)

world = client.get_world()
carla_map = world.get_map()
blueprints = world.get_blueprint_library()
spectator = world.get_spectator()


def spawn_vehicle(world, carla_map, blueprints):
    vehicle_bp = random.choice(blueprints.filter("vehicle.*"))
    spawn_point = random.choice(carla_map.get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    return vehicle, spawn_point


def get_initial_target_waypoint(carla_map, spawn_point):
    current_waypoint = carla_map.get_waypoint(spawn_point.location)
    next_waypoints = current_waypoint.next(10.0)

    if len(next_waypoints) == 0:
        return None, None

    target_waypoint = next_waypoints[0]
    return current_waypoint, target_waypoint


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def compute_steer(vehicle_transform, target_waypoint, vehicle, k=1.0, soft_term=1.0):
    vehicle_location = vehicle_transform.location
    vehicle_yaw_deg = vehicle_transform.rotation.yaw
    vehicle_yaw_rad = math.radians(vehicle_yaw_deg)

    target_transform = target_waypoint.transform
    target_location = target_transform.location
    target_yaw_deg = target_transform.rotation.yaw
    target_yaw_rad = math.radians(target_yaw_deg)

    # 1) heading error
    yaw_error = normalize_angle(target_yaw_deg - vehicle_yaw_deg)
    heading_error_rad = math.radians(yaw_error)

    # 2) approximate front axle position
    wheel_base = 2.5
    front_x = vehicle_location.x + wheel_base * math.cos(vehicle_yaw_rad)
    front_y = vehicle_location.y + wheel_base * math.sin(vehicle_yaw_rad)

    # 3) vector from target waypoint to front axle
    dx = front_x - target_location.x
    dy = front_y - target_location.y

    # 4) road direction unit vector
    path_x = math.cos(target_yaw_rad)
    path_y = math.sin(target_yaw_rad)

    # 5) signed cross-track error
    cross_track_error = -(dx * (-path_y) + dy * path_x)

    # 6) vehicle speed
    velocity = vehicle.get_velocity()
    speed = math.sqrt(
        velocity.x ** 2 +
        velocity.y ** 2 +
        velocity.z ** 2
    )

    # 7) Stanley correction
    cross_track_term = math.atan2(k * cross_track_error, speed + soft_term)

    # 8) final steer
    steer_rad = heading_error_rad + cross_track_term
    steer = max(-1.0, min(1.0, steer_rad))

    print(
        f"vehicle_yaw={vehicle_yaw_deg:.2f}, "
        f"target_yaw={target_yaw_deg:.2f}, "
        f"yaw_error={yaw_error:.2f}, "
        f"cte={cross_track_error:.2f}, "
        f"speed={speed:.2f}, "
        f"steer={steer:.2f}"
    )

    return steer

def distance_to_waypoint(target_waypoint, vehicle):
    return vehicle.get_transform().location.distance(
        target_waypoint.transform.location
    )


def update_spectator(vehicle):
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(z=40),
            carla.Rotation(pitch=-90)
        )
    )


vehicle = None

try:
    vehicle, spawn_point = spawn_vehicle(world, carla_map, blueprints)

    time.sleep(0.2)

    print("spawn point:", spawn_point.location)
    print("vehicle actual:", vehicle.get_transform().location)
    print("vehicle rotation:", vehicle.get_transform().rotation)

    current_waypoint, target_waypoint = get_initial_target_waypoint(carla_map, spawn_point)

    if current_waypoint is None or target_waypoint is None:
        print("No valid target waypoint found.")
    else:
        print("current waypoint:", current_waypoint.transform.location)
        print("target waypoint:", target_waypoint.transform.location)

        for _ in range(3000):
            distance = distance_to_waypoint(target_waypoint, vehicle)

            if distance < 4.0:
                next_waypoints = target_waypoint.next(10.0)
                if len(next_waypoints) == 0:
                    print("No more next waypoints.")
                    break
                target_waypoint = next_waypoints[0]
                print("update target waypoint:", target_waypoint.transform.location)

            vehicle_transform = vehicle.get_transform()
            steer = compute_steer(vehicle_transform, target_waypoint, vehicle)

            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=0.35,
                    steer=steer,
                    brake=0.0
                )
            )

            vehicle_location = vehicle.get_transform().location
            waypoint_location = target_waypoint.transform.location

            print("vehicle:", vehicle_location)
            print("waypoint:", waypoint_location)
            print("distance:", distance)

            world.debug.draw_point(
                waypoint_location,
                size=0.15,
                color=carla.Color(255, 0, 0),
                life_time=0.1
            )

            update_spectator(vehicle)
            time.sleep(0.05)

finally:
    if vehicle is not None:
        vehicle.destroy()