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
    await drone.connect(system_address="serial:///dev/ttyHS1:2000000")

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
    await asyncio.sleep(10)

    print("-- Go 1m North, 0m East, -1m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.4, 0.0, -1.0, 90.0))
    await asyncio.sleep(5)

    print("-- Go 1m North, 1m East, -1m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.45, 2.9, -1.0, 90.0))
    await asyncio.sleep(7)

    print("-- Go 0m North, 1m East, -1m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(6.2, 5, -1.0, 30.0))
    await asyncio.sleep(10)
    
    print("-- Go 0m North, 0m East, -1m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(9.8, 5, -1.0, 0))
    await asyncio.sleep(10)
    
    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.5, 6.3, -1.0, 45))
    await asyncio.sleep(10)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.7, 10.0, -1.0, 90))
    await asyncio.sleep(10)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.7, 10.0, -1.0, 90.0))
    await asyncio.sleep(3)

    print("-- Go 0m North, 0m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone.offboard.set_position_ned(
            PositionNedYaw(12.1, 13.0, -1.0, 90.0))
    await asyncio.sleep(5)

    print("-- Back Home")
    
    print("1")
    await drone.offboard.set_position_ned(
            PositionNedYaw(12.1, 13.0, -1.0, 270.0))
    await asyncio.sleep(3)

    print("2")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.7, 10.0, -1.0, 270.0))
    await asyncio.sleep(10)

    print("3")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.7, 10.0, -1.0, 270.0))
    await asyncio.sleep(3)

    print("4")
    await drone.offboard.set_position_ned(
            PositionNedYaw(11.5, 6.3, -1.0, 200.0))
    await asyncio.sleep(7)

    print("5")
    await drone.offboard.set_position_ned(
            PositionNedYaw(9.8, 5, -1.0,  180.0))
    await asyncio.sleep(7)
    
    print("6")
    await drone.offboard.set_position_ned(
            PositionNedYaw(6.2, 5, -1.0, 195.0))
    await asyncio.sleep(7)
    
    print("7")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.45, 2.9, -1.0, 270.0))
    await asyncio.sleep(7)

    print("8")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.4, 0.0, -1.0, 270.0))
    await asyncio.sleep(7)

    print("9")
    await drone.offboard.set_position_ned(
            PositionNedYaw(4.4, 0.0, -1.0, 180.0))
    await asyncio.sleep(5)

    print("10")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -1.0, 360.0))
    await asyncio.sleep(5)

    #print("-- Go 0m North, 0m East, 0m Down \
    #        within local coordinate system, turn to face South")
    #await drone.offboard.set_position_ned(
    #        PositionNedYaw(11.9, 14.0, 0.3, 90.0))
    #await asyncio.sleep(10)




    print("-- Stopping offboard")

    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()




    print("-- Stopping offboard")

    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
