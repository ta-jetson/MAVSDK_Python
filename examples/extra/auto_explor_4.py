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

async def obstacle_avoidance(drone, total_distance, depth_threshold):
    """Move the drone a total distance, avoiding obstacles as needed."""
    distance_moved = 0
    consecutive_turns = 0
    yaw = 0  # Initial heading

    while distance_moved < total_distance:
        min_depth_cm = await get_min_depth()

        if min_depth_cm <= depth_threshold:
            print("Obstacle detected! Turning right.")
            yaw += 90
            if yaw >= 360:
                yaw -= 360  # Keep yaw within 0-360 degrees

            consecutive_turns += 1

            if consecutive_turns >= 2:
                print("Two consecutive obstacles detected. Turning around.")
                yaw += 180
                if yaw >= 360:
                    yaw -= 360
                consecutive_turns = 0
                await asyncio.sleep(2)

                # Check again after 180-degree turn
                min_depth_cm = await get_min_depth()
                if min_depth_cm <= depth_threshold:
                    # Obstacle still detected, apply left-turn maneuver
                    print("Still obstacle ahead after 180 turn, taking left maneuver.")
                    
                    # Step 1: Turn left 90 degrees
                    yaw -= 90
                    if yaw < 0:
                        yaw += 360
                    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                    await asyncio.sleep(2)

                    # Step 2: Move 1.5 meters forward in the new direction
                    await move_forward(drone, 1.5, yaw)
                    
                    # Step 3: Turn left again (another 90 degrees)
                    yaw -= 90
                    if yaw < 0:
                        yaw += 360
                    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                    await asyncio.sleep(2)

                    # Step 4: Check again for obstacle
                    min_depth_cm = await get_min_depth()
                    if min_depth_cm > depth_threshold:
                        print("Clear path after left maneuver. Aligning and moving forward.")
                        # Turn left to align with original path and continue
                        yaw -= 90
                        if yaw < 0:
                            yaw += 360
                        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                    else:
                        # Obstacle still present; continue exploration pattern with alternating turns
                        print("Exploration mode: alternating left and right turns.")
                        while True:
                            # Alternating left and right turns
                            yaw += 90 if (consecutive_turns % 2 == 0) else -90
                            if yaw < 0:
                                yaw += 360
                            elif yaw >= 360:
                                yaw -= 360
                            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
                            await move_forward(drone, 0.5, yaw)  # Small forward step
                            min_depth_cm = await get_min_depth()
                            if min_depth_cm > depth_threshold:
                                print("Path cleared, resuming forward movement.")
                                break
                            consecutive_turns += 1

        else:
            # Move forward 1 meter in the current yaw direction
            await move_forward(drone, 1, yaw)
            distance_moved += 1
            consecutive_turns = 0  # Reset consecutive turns if moving forward

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

    # Define total distance to move and depth threshold
    total_distance = 10  # Distance in meters to move forward
    depth_threshold = STOP_THRESHOLD_CM  # Stop if an obstacle is within 50 cm

    # Perform obstacle-avoiding forward movement
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

