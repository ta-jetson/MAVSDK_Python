#!/usr/bin/env python3

import asyncio
import numpy as np
import pyrealsense2 as rs
import math
import sensor_msgs.point_cloud2 as pc2
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
import rospy
from sensor_msgs.msg import PointCloud2

# Configuration Parameters
TARGET_HFOV_DEG = 45
NUM_POINTS = 5
MIN_DEPTH_M = 0.25
MAX_DEPTH_M = 6.0
STOP_THRESHOLD_CM = 50

# Initialize global variable for storing the depth data
global_depth_data = None

def point_cloud_callback(msg):
    global global_depth_data
    # Convert the PointCloud2 message to a numpy array
    pc_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    # Filtering the points within the depth range
    valid_points = pc_array[(pc_array[:, 2] > MIN_DEPTH_M) & (pc_array[:, 2] < MAX_DEPTH_M)]

    # Update the global depth data
    global_depth_data = valid_points

async def get_min_depth():
    global global_depth_data
    if global_depth_data is None or len(global_depth_data) == 0:
        return None

    # Calculate horizontal field of view to determine sector indices
    total_points = global_depth_data.shape[0]
    sector_step = total_points // NUM_POINTS

    min_distances = []
    for i in range(NUM_POINTS):
        sector_points = global_depth_data[i * sector_step: (i + 1) * sector_step, 2]
        if len(sector_points) > 0:
            min_distances.append(np.min(sector_points))

    if len(min_distances) == 0:
        return 0

    return np.min(min_distances) * 100  # Convert to cm

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
    # Initialize the ROS node
    rospy.init_node('drone_collision_prevention')

    # Subscribe to the point cloud topic
    rospy.Subscriber("/stereo_pc", PointCloud2, point_cloud_callback)

    # Run the asyncio loop
    asyncio.run(run())
