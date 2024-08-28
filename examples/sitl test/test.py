async def collision_prevention(drone, waypoints, depth_threshold, landing_phase=False):
    """Moves the drone to the target position while preventing collisions and scales velocity."""

    max_velocity = 1.0  # Maximum velocity in m/s
    min_velocity = 0.1  # Minimum velocity in m/s
    slowdown_distance = 0.5  # Distance from stop threshold where scaling starts

    for waypoint in waypoints:
        target_north, target_east, target_down, target_yaw, delay = waypoint
        obstacle_detected = False

        while True:
            min_depth_cm = await get_min_depth()
            current_position = await get_current_position(drone)

            # Print the minimum depth data
            print(f"Minimum depth detected: {min_depth_cm} cm")

            if min_depth_cm <= depth_threshold and not landing_phase:
                # If collision is detected and not in landing phase, hold the current position
                print("Collision detected! Holding position.")
                obstacle_detected = True
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, target_yaw))
            else:
                # If obstacle was detected earlier, wait for 2 seconds before moving again
                if obstacle_detected:
                    print("Obstacle removed! Waiting for 2 seconds before continuing.")
                    await asyncio.sleep(2)
                    obstacle_detected = False

                # Calculate velocity based on distance to the stop threshold
                distance_to_stop_threshold = (min_depth_cm / 100.0) - depth_threshold  # Convert cm to meters
                if distance_to_stop_threshold > slowdown_distance:
                    # Move at maximum velocity
                    velocity = max_velocity
                else:
                    # Start velocity scaling
                    scaling_distance = slowdown_distance
                    velocity_factor = distance_to_stop_threshold / scaling_distance
                    velocity = min_velocity + (max_velocity - min_velocity) * (1 - (1 - velocity_factor))

                # Ensure the velocity is within the specified range
                velocity = max(min_velocity, min(velocity, max_velocity))

                # Continue moving towards the target position
                print(f"Moving to position: North: {target_north} m, East: {target_east} m, Down: {target_down} m with velocity {velocity:.2f} m/s")
                await drone.offboard.set_velocity_ned(VelocityNedYaw(
                    velocity * (target_north - current_position[0]),
                    velocity * (target_east - current_position[1]),
                    velocity * (target_down - current_position[2]),
                    target_yaw
                ))

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
    await drone.connect(system_address="udp://:14540")

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
        (0.0, 0.0, -1.0, 0.0, 4),
        (7, 0.0, -1.0, 0.0, 4),
        (7, 0.0, -1.0, 180.0, 4),
        (0, 0, -1.0, 180.0, 4),
        (0, 0, 0, 180, 4)
    ]

    # Define depth threshold (in centimeters) for collision prevention
    depth_threshold = STOP_THRESHOLD_CM  # Stop if an obstacle is within 100 cm

    # Perform collision-prevented movement
    await collision_prevention(drone, waypoints, depth_threshold, landing_phase=False)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()

    # Add a delay to ensure the drone has landed
    await asyncio.sleep(5)

    # Perform collision-prevented movement during landing
    print("-- Performing collision prevention during landing")
    await collision_prevention(drone, [], depth_threshold, landing_phase=True)

    print("-- Disarming")
    await drone.action.disarm()

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
