#!/usr/bin/env python3

import asyncio
import numpy as np
import pyrealsense2 as rs
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

# Configuration Parameters
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30
TARGET_HFOV_DEG = 45
NUM_POINTS = 5
MIN_DEPTH_M = 0.25
MAX_DEPTH_M = 6.0
STOP_THRESHOLD_CM = 50

pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
profile = pipe.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().intrinsics
depth_hfov_deg = math.degrees(2 * math.atan(DEPTH_WIDTH / (2 * depth_intrinsics.fx)))
angle_offset = (depth_hfov_deg - TARGET_HFOV_DEG) / 2
sector_step = DEPTH_WIDTH * (TARGET_HFOV_DEG / depth_hfov_deg) / NUM_POINTS
start_index = int(DEPTH_WIDTH / 2 - (DEPTH_WIDTH * TARGET_HFOV_DEG / depth_hfov_deg) / 2)

async def get_min_depth():
    frames = pipe.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return None

    depth_image = np.asanyarray(depth_frame.get_data())
    distances = np.full(NUM_POINTS, 65535, dtype=np.uint16)
    for i in range(NUM_POINTS):
        segment_center = int(start_index + i * sector_step)
        if segment_center < 0 or segment_center >= DEPTH_WIDTH:
            continue

        sector_data = depth_image[:, segment_center]
        valid_depths = sector_data[sector_data > 0] * depth_scale
        if valid_depths.size > 0:
            min_depth = np.min(valid_depths)
            if MIN_DEPTH_M < min_depth < MAX_DEPTH_M:
                distances[i] = int(min_depth * 100)  # Convert to cm

    valid_distances = distances[distances != 65535]
    return np.min(valid_distances) if valid_distances.size > 0 else 0

async def move_forward(drone, distance, yaw):
    print(f"Moving forward {distance} meters with yaw {yaw}")
    await drone.offboard.set_position_ned(PositionNedYaw(distance, 0.0, 0.0, yaw))
    await asyncio.sleep(distance)

async def obstacle_avoidance(drone, total_distance, depth_threshold):
    distance_moved = 0
    consecutive_turns = 0
    yaw = 0  # Initial heading

    while distance_moved < total_distance:
        min_depth_cm = await get_min_depth()

        if min_depth_cm <= depth_threshold:
            print("Obstacle detected! Turning right.")
            yaw += 90
            if yaw >= 360:
                yaw -= 360

            consecutive_turns += 1

            if consecutive_turns >= 2:
                print("Two consecutive obstacles detected. Turning around.")
                yaw += 180
                if yaw >= 360:
                    yaw -= 360
                consecutive_turns = 0
                await asyncio.sleep(2)

                min_depth_cm = await get_min_depth()
                if min_depth_cm <= depth_threshold:
                    print("Still obstacle ahead after 180 turn, taking left maneuver.")
                    
                    yaw -= 90
                    if yaw < 0:
                        yaw += 360
                    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                    await asyncio.sleep(2)

                    await move_forward(drone, 1.5, yaw)
                    
                    yaw -= 90
                    if yaw < 0:
                        yaw += 360
                    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                    await asyncio.sleep(2)

                    min_depth_cm = await get_min_depth()
                    if min_depth_cm > depth_threshold:
                        print("Clear path after left maneuver. Aligning and moving forward.")
                        yaw -= 90
                        if yaw < 0:
                            yaw += 360
                        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                    else:
                        print("Exploration mode: alternating left and right turns.")
                        while True:
                            yaw += 90 if (consecutive_turns % 2 == 0) else -90
                            if yaw < 0:
                                yaw += 360
                            elif yaw >= 360:
                                yaw -= 360
                            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                            await move_forward(drone, 0.5, yaw)
                            min_depth_cm = await get_min_depth()
                            if min_depth_cm > depth_threshold:
                                print("Path cleared, resuming forward movement.")
                                break
                            consecutive_turns += 1

        else:
            await move_forward(drone, 1, yaw)
            distance_moved += 1
            consecutive_turns = 0

        await asyncio.sleep(0.1)

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyTHS0:3000000")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a local position estimate...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("-- Local position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        await drone.action.disarm()
        return

    total_distance = 10.0  # Set total distance to travel
    depth_threshold = STOP_THRESHOLD_CM

    await obstacle_avoidance(drone, total_distance, depth_threshold)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()
    await asyncio.sleep(5)
    print("-- Disarming")
    await drone.action.disarm()

if __name__ == "__main__":
    asyncio.run(run())

