#!/usr/bin/env python3

"""
Caveat when attempting to run the examples in non-gps environments:

`drone.offboard.stop()` will return a `COMMAND_DENIED` result because it
requires a mode switch to HOLD, something that is currently not supported in a
non-gps environment.
"""

import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyTHS0:3000000")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go 0m North, 0m East, -1m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    await asyncio.sleep(3)

    print("-- Go 1m North, 0m East, -1m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.4, 0.0, -1.0, 0.0))
    await asyncio.sleep(7)

    print("-- Go 1m North, 0m East, -1m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.4, 0.0, -1.0, 90.0))
    await asyncio.sleep(4)

    print("-- Go 1m North, 1m East, -1m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.45, 2.9, -1.0, 90.0))
    await asyncio.sleep(5)

    print("-- Go 0m North, 1m East, -1m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(5.9, 5.0, -1.0, 30.0))
    await asyncio.sleep(8)
    
    print("-- Go 0m North, 0m East, -1m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(10.1, 4.8, -1.0, 0))
    await asyncio.sleep(6)
    
    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.9, 6.6, -1.0, 45))
    await asyncio.sleep(4)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.9, 6.6, -1.0, 90))
    await asyncio.sleep(2)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(12.3, 10.0, -1.0, 90.0))
    await asyncio.sleep(5)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(12.7, 13.0, -1.0, 90.0))
    await asyncio.sleep(5)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(12.7, 13.0, 0.3, 90.0))
    await asyncio.sleep(1)

    print("-- Stopping offboard")

    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
