import logging
import threading
import time

from Support.Functions import establish_connection_with_vehicles
from LF_Implementations.Proposed_Model.models.node_rev import Node
from LF_Implementations.Proposed_Model.parameters.operational_parameters import OperationalParameters

system_parameters = OperationalParameters.get_instance()

logging.basicConfig(
    format="%(asctime)s: Entry: %(message)s",
    level=logging.INFO,
    datefmt="%H:%M:%S",
)


def rev_main():
    vehicle_connection_strings = [
        "127.0.0.1:14011", "127.0.0.1:14021", "127.0.0.1:14031", "127.0.0.1:14041", "127.0.0.1:14051"
    ]
    drone_objects = establish_connection_with_vehicles(
        vehicle_connection_strings=vehicle_connection_strings
    )

    logging.info("Connection established with drones successfully.")

    # define the function to fly the drones
    def fly_drone(node_id, drone_object):
        if not drone_object.is_armed():
            while not drone_object.is_ready_to_arm():
                time.sleep(1)
            drone_object.arm_the_vehicle()
        logging.info("Drone {} armed successfully.".format(node_id))
        drone_object.takeoff_the_drone(altitude_expected=10)
        logging.info("Drone {} took off successfully.".format(node_id))

    # do the task in threaded form
    threads = []
    for i, do in enumerate(drone_objects):
        p = threading.Thread(target=fly_drone, args=(i+1, do))
        p.start()
        threads.append(p)
    for p in threads:
        p.join()

    # create nodes
    a1 = Node(1, "leader", drone_objects[0], 0, 0)
    a2 = Node(2, "follower", drone_objects[1], 0.00003, 20)
    a3 = Node(3, "follower", drone_objects[2], 0.00003, 40)
    a4 = Node(4, "follower", drone_objects[3], 0.00003, 60)
    a5 = Node(5, "follower", drone_objects[4], 0.00003, 80)

    node_list = [a1, a2, a3, a4, a5]

    logging.info("Mission started")

    # come to the initial formation
    for i, node in enumerate(node_list):
        # ignore leader
        if i == 0:
            continue

        # get leader position and calculate new point to move, then make the move
        node.current_leader_position = a1.dronekit_object.get_current_location()
        new_point_to_move = node.calculate_new_coordinate_to_move()
        node.dronekit_object.move_to_a_point(
            point_coordinates=new_point_to_move,
            sleep_time=0.1,
            ground_speed=7.5,
            tracking_need=True
        )

    logging.info("Initial formation established")

    # start operation
    for node in node_list:
        node.start_operation()
