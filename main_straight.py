import carla
import random
import time

client = carla.Client("localhost", 2000)
client.set_timeout(5.0)

world = client.get_world()
blueprints = world.get_blueprint_library()

# 随机选一辆车；如果你想固定车型，后面可以改成 blueprints.find(...)
vehicle_bp = random.choice(blueprints.filter("vehicle.*"))
#获得所有的出生地点
spawn_points = world.get_map().get_spawn_points()
#随机挑选一个出生地点
spawn_point = random.choice(spawn_points)

#选了一辆车和一个出生地点，进行
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
print("Vehicle spawned")

try:
    spectator = world.get_spectator()

    # 运行 20 秒
    for _ in range(2000):
        #获取车辆的位置
        transform = vehicle.get_transform()

        # 让大窗口镜头跟在车后上方
        spectator.set_transform(
            carla.Transform(
                transform.location + carla.Location(x=-8, z=4),
                carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
            )
        )

        # 给车油门，让它直行
        control = carla.VehicleControl(
            throttle=0.4,
            steer=0.0,
            brake=0.0
        )
        vehicle.apply_control(control)

        time.sleep(0.01)

finally:
    vehicle.destroy()
    print("Vehicle destroyed")