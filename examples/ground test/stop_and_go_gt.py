#!/usr/bin/env python3

import asyncio
import numpy as np
import pyrealsense2 as rs
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# Configuration Parameters
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30
TARGET_HFOV_DEG = 45
NUM_POINTS = 5
MIN_DEPTH_M = 0.25
MAX_DEPTH_M = 6.0
STOP_THRESHOLD_CM = 100

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

    return np.min(distances[distances != 65535])

async def get_current_position(drone):
    """Retrieve the current local position and yaw of the drone."""
    async for position in drone.telemetry.position_velocity_ned():
        return position.position.north_m, position.position.east_m, position.position.down_m
async def collision_prevention(drone, target_position, depth_threshold):
    """ Moves the drone to the target position while preventing collisions. """
    
    for i in enumerate(way)

    # Retrieve the target position values
    target_north, target_east, target_down = target_position

    # Get initial target yaw from the target position
    initial_yaw = 0.0

    while True:
        min_depth_cm = await get_min_depth()
        if min_depth_cm is None:
            print("Depth data unavailable. Continuing.")
            await asyncio.sleep(0)
            continue

        print(f"Minimum Depth: {min_depth_cm:.2f} cm")

        if min_depth_cm <= depth_threshold:
            # If collision is detected, hold the current position
            current_position = await get_current_position(drone)
            print("Collision detected! Holding position.")
            await drone.offboard.set_position_ned(
                PositionNedYaw(current_position[0], current_position[1], current_position[2], initial_yaw))
        else:
            # Continue moving towards the target position
            print(f"Moving to position: North: {target_north} m, East: {target_east} m, Down: {target_down} m")
            await drone.offboard.set_position_ned(
                PositionNedYaw(target_north, target_east, target_down, initial_yaw))

        await asyncio.sleep(0)  # Adjust the check frequency as needed

async def run():
    """ Connects to the drone and performs collision-prevented movement. """

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

    """
    print("-- Arming")
    #await drone.action.arm()

    #print("-- Setting initial setpoint")
    #await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
        
    """

    # Define target position (NED coordinates)
    target_position = (5.0, 0.0, -1.0)  # Move 5m North, maintain altitude at -1m (1m above the ground)

    # Define depth threshold (in centimeters) for collision prevention
    depth_threshold = STOP_THRESHOLD_CM  # Stop if an obstacle is within 200 cm

    # Perform collision-prevented movement
    await collision_prevention(drone, target_position, depth_threshold)
"""
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()
"""
if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
