#!/usr/bin/env python3

import asyncio
from mavsdk import System

async def print_current_local_position(drone):
    """ Continuously prints the current position of the drone in NED coordinates. """
    async for position in drone.telemetry.position_velocity_ned():
        north = position.position.north_m
        east = position.position.east_m
        down = position.position.down_m
        print(f"Current Local Position -> North: {north:.2f} m, East: {east:.2f} m, Down: {down:.2f} m")
        await asyncio.sleep(0)  # Adjust the frequency of updates if needed

async def run():
    """ Connects to the drone and prints its local position. """

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

    # Start printing the current local position
    await print_current_local_position(drone)

if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

