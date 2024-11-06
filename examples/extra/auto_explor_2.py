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
    if valid_distances.size == 0:
        return 0

    return np.min(valid_distances)

async def move_forward(drone, distance, yaw=0):
    """Move forward a specific distance in meters with a given yaw angle."""
    current_position = await get_current_position(drone)
    target_north = current_position[0] + distance * math.cos(math.radians(yaw))
    target_east = current_position[1] + distance * math.sin(math.radians(yaw))
    await drone.offboard.set_position_ned(PositionNedYaw(target_north, target_east, current_position[2], yaw))

    while True:
        current_position = await get_current_position(drone)
        if (abs(current_position[0] - target_north) < 0.05 and
            abs(current_position[1] - target_east) < 0.05):
            break
        await asyncio.sleep(0.1)

async def obstacle_avoidance(drone, depth_threshold):
    while True:
        min_depth_cm = await get_min_depth()

        if min_depth_cm <= depth_threshold:
            print("Obstacle detected! Turning right and sidestepping.")

            # Turn 90 degrees right
            current_position = await get_current_position(drone)
            await drone.offboard.set_position_ned(PositionNedYaw(current_position[0], current_position[1], current_position[2], 90))
            await asyncio.sleep(2)

            # Move 1 meter to the right
            await move_forward(drone, 1, yaw=90)

            # Turn back to original heading (0 degrees)
            await drone.offboard.set_position_ned(PositionNedYaw(current_position[0], current_position[1], current_position[2], 0))
            await asyncio.sleep(2)

            print("Sidestep complete, resuming forward motion.")
        else:
            # Move forward 1 meter at a time
            await move_forward(drone, 1)

        await asyncio.sleep(0.1)

async def run():
    """ Connects to the drone and starts obstacle-avoiding forward movement. """

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
        print("-- Disarming")
        await drone.action.disarm()
        return

    depth_threshold = STOP_THRESHOLD_CM
    await obstacle_avoidance(drone, depth_threshold)

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

