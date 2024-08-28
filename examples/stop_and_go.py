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

    # Setting invalid measurements as None
    valid_distances = distances[distances != 65535]
    if valid_distances.size == 0:
        return 0

    return np.min(valid_distances)

async def get_current_position(drone):
    """Retrieve the current local position and yaw of the drone."""
    async for position in drone.telemetry.position_velocity_ned():
        return position.position.north_m, position.position.east_m, position.position.down_m

async def collision_prevention(drone, waypoints, depth_threshold):
    """ Moves the drone to the target position while preventing collisions. """

    for waypoint in waypoints:
        # Retrieve the target position values
        target_north, target_east, target_down, target_yaw, delay = waypoint

        while True:
            min_depth_cm = await get_min_depth()
            current_position = await get_current_position(drone)

            if min_depth_cm <= depth_threshold:
                # If collision is detected, hold the current position
                print("Collision detected! Holding position.")
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, target_yaw))
            else:
                # Continue moving towards the target position
                print(f"Moving to position: North: {target_north} m, East: {target_east} m, Down: {target_down} m")
                await drone.offboard.set_position_ned(
                    PositionNedYaw(target_north, target_east, target_down, target_yaw))

                # Check if the drone has reached the target position
                if (abs(current_position[0] - target_north) < 0.05 and
                    abs(current_position[1] - target_east) < 0.05 and
                    abs(current_position[2] - target_down) < 0.05):
                    print("Reached the target position.")
                    break

            await asyncio.sleep(0.01)  # Adjust the check frequency as needed

        print(f"Holding position for {delay} seconds.")
        await asyncio.sleep(delay)

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

    # Define waypoints with NED coordinates, yaw, and delay (seconds)
    waypoints = [
        (0.0, 0.0, -1.0, 0.0, 3),
        (4.4, 0.0, -1.0, 0.0, 7),
        (4.4, 0.0, -1.0, 90.0, 8),
        (4.45, 2.9, -1.0, 90.0, 5),
        (5.9, 5.0, -1.0, 30.0, 8),
        (10.1, 4.8, -1.0, 0.0, 6),
        (11.9, 6.6, -1.0, 45.0, 4),
        (11.9, 6.6, -1.0, 90.0, 2),
        (12.3, 10.0, -1.0, 90.0, 5),
        (12.7, 13.0, -1.0, 90.0, 5),
        (12.7, 13.0, 0.3, 90.0, 1)
    ]

    # Define depth threshold (in centimeters) for collision prevention
    depth_threshold = STOP_THRESHOLD_CM  # Stop if an obstacle is within 50 cm

    # Perform collision-prevented movement
    await collision_prevention(drone, waypoints, depth_threshold)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()

    # Add a delay to ensure the drone has landed
    await asyncio.sleep(5)

    print("-- Disarming")
    await drone.action.disarm()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

