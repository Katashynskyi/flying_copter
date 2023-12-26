import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import logging as logger
import argparse

logger.getLogger().setLevel(logger.INFO)


def arm_and_takeoff(target_alt: int) -> None:
    """
    Arms the vehicle and takes off to the specified altitude.

    :param target_alt: The target altitude for takeoff.
    :return: None
    """

    logger.info("Arming motors")

    vehicle.armed = True

    while not vehicle.armed:
        logger.info(" Waiting for arming...")
        time.sleep(1)
        vehicle.armed = True

    logger.info("Taking off!")
    while True:
        if vehicle.location.global_relative_frame.alt < target_alt * 0.9:
            vehicle.channels.overrides["3"] = 2200
        elif vehicle.location.global_relative_frame.alt > target_alt * 0.9:
            vehicle.channels.overrides["3"] = 1750
        if vehicle.location.global_relative_frame.alt >= target_alt * 0.995:
            logger.info(
                f"Reached target altitude of {int(vehicle.location.global_relative_frame.alt)} m"
            )
            vehicle.channels.overrides["3"] = 1500
            break
        time.sleep(1)


def get_distance_mtr(location1, location2) -> float:
    """
    Calculates the distance between two geographic locations.

    :param location1: The first geographic location.
    :param location2: The second geographic location.
    :return: The distance between the two locations in meters.
    """
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_bearing(location1, location2) -> float:
    """
    Calculates the bearing (azimuth) between two geographic locations.

    :param location1: The first geographic location.
    :param location2: The second geographic location.
    :return: The bearing (azimuth) in degrees.
    """
    off_x = location2.lon - location1.lon
    off_y = location2.lat - location1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def way_to_point(wpl) -> None:
    """
    Navigates the vehicle to a specific geographic point.

    :param wpl: The target geographic location.
    :return: None
    """
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.channels.overrides["2"] = 1000
    distance_new = int(get_distance_mtr(vehicle.location.global_frame, wpl))
    distance_old = distance_new + 1
    while True:
        if distance_new > distance_old:
            vehicle.channels.overrides["2"] = 1500
            time.sleep(2)
            yaw_needs = get_bearing(vehicle.location.global_frame, wpl)
            condition_yaw(yaw_needs)
            return way_to_point(wpl)
        if distance_new > 300:
            time.sleep(5)
            distance_old = distance_new
            distance_new = int(get_distance_mtr(vehicle.location.global_frame, wpl))
            logger.info(f"Meters left: {distance_new}m")
        elif 300 >= distance_new > 50:
            vehicle.channels.overrides["2"] = 1200
            time.sleep(5)
            distance_old = distance_new
            distance_new = int(get_distance_mtr(vehicle.location.global_frame, wpl))
            logger.info(f"Meters left: {distance_new}m")
        elif 50 >= distance_new >= 3:
            vehicle.channels.overrides["2"] = 1400
            time.sleep(2)
            distance_old = distance_new
            distance_new = int(get_distance_mtr(vehicle.location.global_frame, wpl))
            logger.info(f"Meters left: {distance_new}m")
        else:
            logger.info("Drone at the destination")
            vehicle.channels.overrides["2"] = 1500
            break


def condition_yaw(yaw_need: float) -> None:
    """
    Adjusts the drone's yaw (orientation) to a specific angle.

    :param yaw_need: The desired yaw angle in degrees.
    :return: None
    """
    while int(vehicle.heading) != int(yaw_need):
        if vehicle.heading < yaw_need:
            vehicle.channels.overrides["4"] = 1550
        else:
            vehicle.channels.overrides["4"] = 1450
        time.sleep(0.5)

    vehicle.channels.overrides["4"] = 1500
    logger.info(f"Azimuth:{vehicle.heading} degree")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("--connect", default="udp:127.0.0.1:14551")
    parser.add_argument(
        "--target_altitude", default=100, help="choose altitude to fly in meters"
    )
    parser.add_argument(
        "--target_point",
        default=LocationGlobalRelative(50.443326, 30.448078),
        help="Final point to fly to",
    )
    parser.add_argument("--final_yaw", default=350)
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    if not connection_string:
        import dronekit_sitl

        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    logger.info("Connecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    while not vehicle.mode == "ALT_HOLD":
        logger.info(" Waiting for ALT_HOLD mode...")
        time.sleep(1)
    arm_and_takeoff(args.target_altitude)
    yaw = get_bearing(vehicle.location.global_frame, args.target_point)
    condition_yaw(yaw)
    way_to_point(args.target_point)
    condition_yaw(args.final_yaw)
    time.sleep(5)
    logger.info("Mission to Complete")
    logger.info(f" Final Azimuth:{vehicle.heading} degree")
    logger.info(f"Vehicle mode: {vehicle.mode}")
    vehicle.close()
    if sitl:
        sitl.stop()
