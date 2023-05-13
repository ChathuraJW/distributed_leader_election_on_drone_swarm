import logging
import threading
import concurrent.futures
import time
from typing import List

from dronekit import connect

from Support.Drone import Drone
from Support.Formation import Formation
from Support.Parameters import Parameters
from Support.Utility import init_formation_thread_worker

# define parameter object
operational_parameters = Parameters()


def establish_connection_with_vehicles(vehicle_connection_strings):
    """
    This function will connect to vehicles and wait until the successful connection.
    Parameters:
        vehicle_connection_strings: List of connection strings
    Returns:
        List of Drone objects
    """
    vehicle_objects = []
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = []
        for i, connection_string in enumerate(vehicle_connection_strings):
            future = executor.submit(connect, connection_string, wait_ready=True, timeout=90)
            futures.append(future)

        for i, future in enumerate(concurrent.futures.as_completed(futures)):
            vehicle = future.result()
            drone_object = Drone(dronekit_object=vehicle, vehicle_id=i)
            vehicle_objects.append(drone_object)

    return vehicle_objects


def set_init_formation(drone_objects: List[Drone], base_location):
    """
    This function will be change the vehicles to desired position according to the selected formation.
    Arguments:
        drone_objects: A list of Drone objects which are in the swam
        base_location: Base location latitude and longitude coordinates
    """

    # set up the formation expected
    formation = Formation()
    # the formation can be changed and TODO new methods will be added
    formation.make_arrow_formation_six_vehicles(
        base_coordinate=base_location,
        d=operational_parameters.get_inter_drone_distance()
    )
    formation_points = formation.get_formation_points()

    thread_pool = []
    for i, obj in enumerate(drone_objects):
        thread = threading.Thread(target=init_formation_thread_worker, args=(formation_points, obj, i,))
        thread_pool.append(thread)
        thread.start()

    for thread in thread_pool:
        thread.join()

    logging.info("Vehicles scattered according to the formation.")


def single_vehicle_basic_move(drone_object: Drone):
    """
    This function offers basic navigation for a single drone. This function will be executed below-mentioned operation
    in order.
        1. Check ready to arm
        2. Arm the vehicle
        3. Takeoff
        4. Fly to given locations(circuit)
        5. Land the vehicle
        6. Disarm the vehicle
    Arguments:
        drone_object: Drone object
    """

    # if the vehicle not armed yet
    if not drone_object.is_armed():
        # wait until ready to arm
        while not drone_object.is_ready_to_arm():
            time.sleep(1)
        # arm the vehicle and do takeoff
        drone_object.arm_the_vehicle()
        drone_object.takeoff_the_drone(altitude_expected=20)

    # mission assignment for navigation
    for point in operational_parameters.get_path_points():
        drone_object.move_to_a_point(point_coordinates=point)
        logging.info(
            "Vehicle with VID " + str(drone_object.vehicle_id) + " accepted the mission.(Move to: " + str(point) + ")"
        )

    logging.info(
        "Vehicle with VID " + str(drone_object.vehicle_id) + " completed assigned mission."
    )
    # vehicle landing
    drone_object.land_the_vehicle()
    logging.info(
        "Vehicle with VID " + str(drone_object.vehicle_id) + " landed."
    )
