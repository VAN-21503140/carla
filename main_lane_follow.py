import carla
import random
import time


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


def compute_steer(vehicle_transform, target_waypoint):
    vehicle_yaw = vehicle_transform.rotation.yaw
    target_yaw = target_waypoint.transform.rotation.yaw

    yaw_error = normalize_angle(target_yaw - vehicle_yaw)

    steer = yaw_error / 50.0
    steer = max(-1.0, min(1.0, steer))

    print(
        f"vehicle_yaw={vehicle_yaw:.2f}, "
        f"target_yaw={target_yaw:.2f}, "
        f"yaw_error={yaw_error:.2f}, "
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

    # 等一小会儿，让车辆生成后状态稳定一点
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

        for _ in range(600):
            distance = distance_to_waypoint(target_waypoint, vehicle)

            # 到达目标点附近后，沿着当前目标点继续往前找下一个
            if distance < 4.0:
                next_waypoints = target_waypoint.next(10.0)
                if len(next_waypoints) == 0:
                    print("No more next waypoints.")
                    break
                target_waypoint = next_waypoints[0]
                print("update target waypoint:", target_waypoint.transform.location)

            vehicle_transform = vehicle.get_transform()
            steer = compute_steer(vehicle_transform, target_waypoint)

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

            # 在地图上画出目标 waypoint
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