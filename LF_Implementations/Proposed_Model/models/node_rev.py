from __future__ import annotations

import logging
import math
import random
import threading
import time

import sqlite3
import psutil

from dronekit import LocationGlobalRelative

from LF_Implementations.Proposed_Model.models.communicator_rev import Communicator
from LF_Implementations.Proposed_Model.models.message_rev import Heartbeat, Message, NodeGoingToFail
from LF_Implementations.Proposed_Model.parameters.operational_parameters import OperationalParameters
from LF_Implementations.Proposed_Model.models.election import start_election
from LF_Implementations.Proposed_Model.models.message_processor import process_message
from Support.Drone import Drone

system_parameters = OperationalParameters.get_instance()

communicator = Communicator()
wait_to_fail_leader = 50
wait_to_fail_follower = 100
wait_to_fail_after_inform_failure = 25


class Node:
    def __init__(self, node_id: int, node_flag, dronekit_object, l_value: float, psi_value: float):
        """
        This method is used to initialize the node.
        :param node_id: The id of the node.
        :param node_flag: The flag of the node.
        :param dronekit_object: The drone object of the node.
        :param l_value: The l value of the node.
        :param psi_value: The psi value of the node.
        """
        self.node_id: int = node_id
        self.node_flag = node_flag
        self.dronekit_object: Drone = dronekit_object

        # formation parameters
        self.l_value: float = l_value
        self.psi_value: float = psi_value
        self.am_i_alive = True

        # operating parameters
        self.current_leader_position: dict = None
        self.last_heartbeat_received_at: time = time.time()
        self.leader_rsv: float = 0.0

        # aware message
        self.i_know_leader_is_elected = False
        self.is_mission_complete = False
        self.am_i_alive = True
        self.am_i_going_to_fail = False

        self.is_leader_failed = False
        self.is_there_a_leader_failure_sign = False

        # set current leader to defined parameter initially
        self.current_leader_id: int = system_parameters.initial_leader

        # election parameters
        self.is_election_started = False
        self.rsv_vector = [{"node_id": 0, "rsv": 0.0, "calculated_at": time.time(
        )}] * system_parameters.number_of_nodes
        self.vote_vector = [{"node_id": 0, "rsv": 0.0, "calculated_at": time.time(),
                             "confidence_score": 0.0}] * system_parameters.number_of_nodes

        # make leader's rsv_vector, vote_vector positions None
        self.rsv_vector[self.current_leader_id - 1] = None
        self.vote_vector[self.current_leader_id - 1] = None

        self.election_first_Round_starts_at: time = None
        self.election_second_Round_starts_at: time = None
        self.election_third_Round_starts_at: time = None

        self._my_rsv = 0.0
        self._priority_factor = 0.0
        self._payload_factor = 0.0

        self._accu_counter = 0
        self._parameter_accu_vector = {
            "cpu": 0.0, "memory": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        self.last_heartbeat_received_at = time.time()

        # leaders state variable, which represents the leaders goal
        self.leader_goal_state: LocationGlobalRelative = None

        # update RSV
        self.calculate_rsv()

    def get_my_rsv(self):
        return self._my_rsv

    def clean_rsv_vector(self):
        """
        This method is used to clean the rsv vector. For that, make the node_id parameter a zero.
        """
        for i in range(len(self.rsv_vector)):
            if self.rsv_vector[i] is not None:
                self.rsv_vector[i]["node_id"] = 0
                self.rsv_vector[i]["rsv"] = 0.0
                self.rsv_vector[i]["calculated_at"] = time.time()

    def clean_vote_vector(self):
        """
        This method is used to clean the vote vector. For that, make the node_id parameter a zero.
        """
        for i in range(len(self.vote_vector)):
            if self.vote_vector[i] is not None:
                self.vote_vector[i]["node_id"] = 0
                self.vote_vector[i]["rsv"] = 0.0
                self.vote_vector[i]["calculated_at"] = time.time()
                self.vote_vector[i]["confidence_score"] = 0.0

    def scrape_rsv_parameters(self, need_value_return: bool = False) -> dict | None:
        """
        This method is used to scrape the rsv parameters and update the _parameter_accu_vector. and return those values
        as a  dictionary.
        """
        # get current process, cpu and memory usage
        cpu = psutil.cpu_percent(interval=0)
        memory = psutil.virtual_memory().percent

        battery = self.dronekit_object.get_battery()["level"]
        drone_attitude = self.dronekit_object.get_attitude()

        # consider absolute value due to I need only the magnitude of the value to detect the deviation from the normal
        roll = abs(drone_attitude["roll"])
        pitch = abs(drone_attitude["pitch"])
        yaw = abs(drone_attitude["yaw"])

        # update vector
        self._parameter_accu_vector["cpu"] += cpu
        self._parameter_accu_vector["memory"] += memory
        self._parameter_accu_vector["roll"] += roll
        self._parameter_accu_vector["pitch"] += pitch
        self._parameter_accu_vector["yaw"] += yaw

        # increase counter
        self._accu_counter += 1

        if need_value_return:
            return {
                "counter": self._accu_counter,
                "cpu": cpu, "memory": memory,
                "battery": battery,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw
            }

    def calculate_rsv(self):
        """
        This method is used to calculate the resource strength value of the node.
        """
        # get current reading
        current_reading = self.scrape_rsv_parameters(need_value_return=True)

        cpu_usage_factor = current_reading["cpu"]
        cpu_history_usage_factor = self._parameter_accu_vector["cpu"] / \
            self._accu_counter
        memory_usage_factor = current_reading["memory"]
        memory_history_usage_factor = self._parameter_accu_vector["memory"] / \
            self._accu_counter
        battery_usage_factor = float(current_reading["battery"])

        threshold = system_parameters.max_attitude_threshold

        drone_attitude_history_factor = (2.0 - ((
            (self._parameter_accu_vector["roll"] / self._accu_counter) +
            (self._parameter_accu_vector["pitch"] / self._accu_counter) +
            (self._parameter_accu_vector["yaw"] / self._accu_counter)
        ) / (3 * threshold))) * 50.0
        payload_factor = (
            (1 - (system_parameters.payload_for_each_drone[self.node_id - 1]) /
             (max(system_parameters.payload_for_each_drone) + 0.000025))
        ) * 100.0

        user_defined_priority_factor = system_parameters.assigned_priority_for_drone[
            self.node_id - 1] * 1.0

        self._priority_factor = user_defined_priority_factor
        self._payload_factor = payload_factor

        # calculate final equation
        final_eq = (
            (1.0 * (100.0 - cpu_usage_factor)) +
            (1.0 * (100.0 - cpu_history_usage_factor)) +
            (1.0 * (100.0 - memory_usage_factor)) +
            (1.0 * (100.0 - memory_history_usage_factor)) +
            (1.0 * battery_usage_factor) +
            (1.0 * drone_attitude_history_factor) +
            (1.0 * payload_factor) +
            (1.0 * user_defined_priority_factor)
        ) / 8.0

        # save things to the DB
        sqlite_connection = sqlite3.connect("results/RSV/parameters.db")
        my_cursor = sqlite_connection.cursor()
        table_name = f"data_node_{self.node_id}"
        sql = "INSERT INTO " + table_name + " (cpu_usage_factor, cpu_history_usage_factor, memory_usage_factor, " \
                                            "memory_history_usage_factor, battery_usage_factor, " \
                                            "drone_attitude_history_factor , payload_factor, user_defined_priority_factor, rsv, timestamp) VALUES (" \
                                            f"{cpu_usage_factor}, " \
                                            f"{cpu_history_usage_factor}, " \
                                            f"{memory_usage_factor}, " \
                                            f"{memory_history_usage_factor}, " \
                                            f"{battery_usage_factor}, " \
                                            f"{drone_attitude_history_factor}, " \
                                            f"{payload_factor}, " \
                                            f"{user_defined_priority_factor}, " \
                                            f"{final_eq}, " \
                                            f"{time.time()});"

        my_cursor.execute(sql)
        sqlite_connection.commit()
        sqlite_connection.close()

        self._my_rsv = final_eq

    def calculate_new_coordinate_to_move(self) -> LocationGlobalRelative | None:
        """
        This method is used to calculate the new coordinate to move.
        :return: The new LocationGlobalRelative object to move.
        """
        if self.current_leader_position is None:
            return None

        # calculate new positions based on the equations
        lat = round(
            self.current_leader_position["lat"] + self.l_value * math.sin(self.psi_value), 9)
        lon = round(
            self.current_leader_position["lon"] + self.l_value * math.cos(self.psi_value), 9)
        alt = self.current_leader_position["alt"]

        # create location object to move drone
        return LocationGlobalRelative(lat=lat, lon=lon, alt=alt / 10)

    def _get_max_rsv_node_id(self) -> int:
        """
        This method is used to get the max rsv node id.
        Under general circumstances, the node with the highest rsv value is the leader. But if there are two or more
        nodes with the same rsv value, then the node with the lowest calculated_at value is the leader, if it is not
        sufficient, for comparison, the node with the lowest node id is the leader.
        :return: The max rsv node id.
        """
        max_rsv = 0
        max_rsv_calculated_at: float = 0.0
        max_rsv_node_id = 0
        for node_info in self.rsv_vector:
            if node_info is not None:
                if node_info["rsv"] > max_rsv:
                    max_rsv = node_info["rsv"]
                    max_rsv_node_id = node_info["node_id"]
                    max_rsv_calculated_at = node_info["calculated_at"]
                elif node_info["rsv"] == max_rsv:
                    if node_info["calculated_at"] < max_rsv_calculated_at:
                        max_rsv = node_info["rsv"]
                        max_rsv_node_id = node_info["node_id"]
                        max_rsv_calculated_at = node_info["calculated_at"]
                    if node_info["calculated_at"] == max_rsv_calculated_at:
                        if node_info["node_id"] < max_rsv_node_id:
                            max_rsv = node_info["rsv"]
                            max_rsv_node_id = node_info["node_id"]
                            max_rsv_calculated_at = node_info["calculated_at"]
        return max_rsv_node_id

    def get_max_rsv_having_entry(self):
        max_rsv_having_node_id = self._get_max_rsv_node_id()
        return_entry = self.rsv_vector[0]
        for node_info in self.rsv_vector:
            if node_info is not None:
                if node_info["node_id"] == max_rsv_having_node_id:
                    return_entry = node_info
                    break
        return return_entry

    def get_non_filled_rsv_vector_node_id(self) -> []:
        """
        This method is used to get the non-filled rsv vector node id.
        :return: The non filled rsv vector node id.
        """
        non_filled_rsv_vector_node_id = []
        for i, node_info in enumerate(self.rsv_vector):
            if (node_info is not None) and (node_info["node_id"] == 0):
                non_filled_rsv_vector_node_id.append(i + 1)
        return non_filled_rsv_vector_node_id

    def get_active_node_ids(self) -> []:
        """
        This method is used to get the active node ids.
        :return: The active node ids.
        """
        active_node_ids = []
        for i, node_info in enumerate(self.rsv_vector):
            if node_info is not None:
                active_node_ids.append(i + 1)
        return active_node_ids

    def _start_communication_manager(self):
        """
        This method is used to manage the communication of the node.
        """
        while (not self.is_mission_complete) and self.am_i_alive:
            # receive message
            message = communicator.get_message(node_id=self.node_id)
            if message is not None:
                # handle message
                self._handle_message(message=message)
            time.sleep(0.1)

        logging.info(
            f"Node {self.node_id} is shutting down communication manager.")

    def _handle_message(self, message: Message):
        """
        This method is used to handle the message.
        :param message: The message to be handled.
        """
        process_message(message=message, communicator=communicator, self=self)

    def _start_work(self):
        """
        This method is used to start the work of the node.
        """

        # define and start the process which will update the rsv parameter vector for history values
        def scrape_rsv_data():
            while self.am_i_alive:
                # scrape data
                self.scrape_rsv_parameters(need_value_return=False)
                # save current location
                file = open(str(self.node_id) + "_log.txt", "a")
                data_to_write = self.dronekit_object.get_current_location()
                data_to_write["current_leader"] = self.current_leader_id
                self.calculate_rsv()
                data_to_write["current_rsv"] = self.get_my_rsv()
                # write to file
                file.write(f"{data_to_write}\n")
                file.close()
                time.sleep(system_parameters.rsv_scrape_interval / 1000)

        # start the thread
        rsv_scrape_thread = threading.Thread(
            target=scrape_rsv_data,
            name="rsv_scrape_thread for nid: " + str(self.node_id)
        )
        rsv_scrape_thread.start()
        print("RSV scrape thread started for node: " + str(self.node_id))

        # start general mission
        while self.am_i_alive:
            # assign task according to the role
            if self.node_flag == "leader":
                self._leaders_work()
            elif self.node_flag == "follower":
                self._followers_work()
            else:
                print("Invalid node flag")
                return -1

    def start_operation(self):
        """
        This method is used to start the operation of the node.
        """
        # create two thread for both tasks
        communication_manager_thread = threading.Thread(
            target=self._start_communication_manager)
        work_thread = threading.Thread(target=self._start_work)
        # start the threads
        communication_manager_thread.start()
        work_thread.start()

    def _leaders_work(self):
        """
        This method is used to start the work of the leader.
        """
        logging.info(
            f"Node {self.node_id} started its work as a leader with node list "
            f"{system_parameters.get_active_node_list()}"
        )

        # communication handling

        # drone navigation setup
        def navigate_leader():
            # find leader state and do the respective navigation
            if self.leader_goal_state in system_parameters.path_points_for_leader:
                t = system_parameters.path_points_for_leader.index(
                    self.leader_goal_state)
                if t == system_parameters.get_path_points_for_leader_list_length() - 1:
                    t = 0
            else:
                t = 0
            for q in range(system_parameters.map_rounds):
                # restore leaders state
                while t < system_parameters.get_path_points_for_leader_list_length():
                    if self.am_i_alive:
                        self.leader_goal_state = system_parameters.path_points_for_leader[t]
                        self.dronekit_object.move_to_a_point(
                            point_coordinates=system_parameters.path_points_for_leader[t],
                            ground_speed=5.0,
                            sleep_time=0.1,
                            tracking_need=False
                        )
                        time.sleep(15)
                        t = t + 1
                    else:
                        exit(10)

        # create thread for drone navigation and start it
        drone_navigator = threading.Thread(target=navigate_leader)
        drone_navigator.start()

        # drone driving
        control_timer = time.time()
        while self.am_i_alive:
            # fail when battery get level 0
            if self.dronekit_object.get_battery()["level"] == 0:
                self.am_i_alive = False
                logging.info(
                    "Node {} (leader) is failed due to lower battery level.".format(self.node_id))
                # disarm the drone -> for simulation only
                self.dronekit_object.hover_the_drone()
                self.dronekit_object.land_the_vehicle()
                self.dronekit_object.disarm_the_vehicle()
                self.dronekit_object.close_the_connection()
                exit(20)

            # do leader work
            if random.random() > system_parameters.node_failing_probability:
                # leader's normal work
                # send the heartbeat message
                # calculate rsv
                self.calculate_rsv()

                # based on RSV ready the failing_flag
                if self._my_rsv < system_parameters.rsv_threshold_to_define_leader_failure:
                    failing_flag = True
                else:
                    failing_flag = False

                # create heartbeat message
                heartbeat = Heartbeat(rsv_value=self._my_rsv,
                                      leader_position=self.dronekit_object.get_current_location(),
                                      leader_state=self.leader_goal_state,
                                      fail_flag=failing_flag)
                # create message and broadcast it
                communicator.broadcast_message(
                    sender_id=self.node_id, message=heartbeat)

                # wait for defined time
                time.sleep(system_parameters.heartbeat_interval / 1000)
            else:
                # stop immediate leader failure
                if control_timer + wait_to_fail_leader > time.time():
                    continue

                # leader is failed
                self.am_i_alive = False
                system_parameters.alive_node_tracker[self.node_id - 1] = False
                logging.warning(
                    "Node {} (leader) is failed.".format(self.node_id))
                # disarm the drone -> for simulation only
                self.dronekit_object.hover_the_drone()
                self.dronekit_object.land_the_vehicle()
                self.dronekit_object.disarm_the_vehicle()
                self.dronekit_object.close_the_connection()
                exit(0)

    def _followers_work(self):
        """
        This method is used to start the work of the follower.
        """
        logging.info(
            f"Node {self.node_id} started its work as a follower, under the leadership of {self.current_leader_id}")

        # the goal of control_timer is to stop the immediate node failures, with first two experiments, it ticks outside
        # the main loop, which represents the notion of "from the time, the node is started".
        control_timer = time.time()
        while self.am_i_alive:
            # do followers work
            # if detect leader failure, freeze the drone and start the election
            if ((self.last_heartbeat_received_at * 1000) +
                system_parameters.leader_failure_detect_from_last_heartbeat) \
                    < (time.time() * 1000):
                logging.info(
                    "No heartbeat received from leader. Starting the election by node {}.".format(self.node_id))
                # leader failed, start the election
                start_election(node_object=self, communicator=communicator)
                # clean rsv and vote vectors
                self.clean_rsv_vector()
                self.clean_vote_vector()
                # break the loop
                return
            else:
                # simply follow the leader
                # make the movement
                while True:
                    time.sleep(0.1)
                    new_point_to_move = self.calculate_new_coordinate_to_move()
                    if new_point_to_move is not None:
                        break
                    else:
                        print("new_point_to_move is None")

                self.dronekit_object.move_to_a_point(
                    point_coordinates=new_point_to_move,
                    sleep_time=0.1,
                    ground_speed=8.5
                )

                # fail when battery get level 0
                if self.dronekit_object.get_battery()["level"] == 0:
                    self.am_i_alive = False
                    logging.info(
                        "Node {} (follower) is failed due to lower battery level.".format(self.node_id))
                    # disarm the drone -> for simulation only
                    self.dronekit_object.hover_the_drone()
                    self.dronekit_object.land_the_vehicle()
                    self.dronekit_object.disarm_the_vehicle()
                    self.dronekit_object.close_the_connection()
                    exit(20)

                # do inform failures, if rsv is lower than threshold, inform the swam about it
                if (self._my_rsv < system_parameters.rsv_threshold_to_define_follower_failure) and (
                        not self.am_i_going_to_fail):
                    if not system_parameters.allow_follower_informed_failure:
                        continue
                    # raise going to fail flag
                    self.am_i_going_to_fail = True
                    # start timer to fail
                    timer_to_fail = time.time()
                    # create log entry
                    logging.info(
                        "Node {} is going to fail.(informed)".format(self.node_id))
                    # create going to fail message and broadcast to the swarm
                    going_to_fail_message = NodeGoingToFail(
                        rsv_value=self._my_rsv)
                    communicator.broadcast_message(
                        sender_id=self.node_id, message=going_to_fail_message)

                # failure after informed for the followers
                if self.am_i_going_to_fail:
                    if timer_to_fail + wait_to_fail_after_inform_failure > time.time():
                        # fail the node
                        self.am_i_alive = False
                        logging.info(
                            "Node {} (follower) is failed due to lower RSV.".format(self.node_id))
                        # disarm the drone -> for simulation only
                        self.dronekit_object.hover_the_drone()
                        self.dronekit_object.land_the_vehicle()
                        self.dronekit_object.disarm_the_vehicle()
                        self.dronekit_object.close_the_connection()
                        exit(20)

                # allow immediate leader failure, guard the incident wih if statement as necessary
                if round(random.random(), 2) == system_parameters.node_failing_probability:
                    if not system_parameters.allow_follower_immediate_failure:
                        continue
                    if control_timer + wait_to_fail_follower > time.time():
                        continue
                    self.am_i_alive = False
                    logging.info(
                        "Node {} (follower) is failed.".format(self.node_id))
                    # disarm the drone -> for simulation only
                    self.dronekit_object.hover_the_drone()
                    self.dronekit_object.land_the_vehicle()
                    self.dronekit_object.disarm_the_vehicle()
                    self.dronekit_object.close_the_connection()
                    exit(20)
