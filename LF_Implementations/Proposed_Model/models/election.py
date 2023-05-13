import logging
import time

from LF_Implementations.Proposed_Model.models.communicator_rev import Communicator
from LF_Implementations.Proposed_Model.models.message_rev import ElectionReply, InformWinner, LeadersBroadcast
from LF_Implementations.Proposed_Model.parameters.operational_parameters import OperationalParameters

system_parameters = OperationalParameters.get_instance()


def election_round_one(node_object, communicator: Communicator):
    """
    In this round, each node broadcasts its RSV to all its neighbors.
    """
    # update necessary parameters
    node_object.is_election_started = True
    node_object.i_know_leader_is_elected = False
    # calculate RSV
    node_object.calculate_rsv()
    # broadcast RSV to all neighbors
    election_message = ElectionReply(rsv_value=node_object.get_my_rsv())
    communicator.broadcast_message(message=election_message, sender_id=node_object.node_id)


def election_round_two(node_object, communicator: Communicator):
    # calculate confidence score
    total_active_node_in_community = 0.0
    total_responses = 0.0
    for node_info in node_object.rsv_vector:
        if node_info is not None:
            total_active_node_in_community += 1
            if node_info["node_id"] != 0:
                total_responses += 1

    confidence_score = total_responses / total_active_node_in_community

    # check election replay vector for max RSV having node
    max_rsv_having_node = node_object.get_max_rsv_having_entry()

    # broadcast winner message
    winner_message = InformWinner(
        winner_id=max_rsv_having_node["node_id"],
        winner_rsv=max_rsv_having_node["rsv"],
        confidence=confidence_score
    )
    communicator.broadcast_message(message=winner_message, sender_id=node_object.node_id)


def election_round_three(node_object, communicator: Communicator):
    vote_count = [[i + 1, 0.0] for i in range(system_parameters.number_of_nodes)]
    # check vote_vector for max vote having node
    for vote_entry in node_object.vote_vector:
        if vote_entry is not None:
            vote_count[vote_entry["node_id"] - 1][1] = \
                vote_count[vote_entry["node_id"] - 1][1] + vote_entry["confidence_score"]

    # get max vote having entry
    max_vote_having_entry_node_id = max(vote_count, key=lambda x: x[1])[0]

    # logging.info("Election result: Node " + str(max_vote_having_entry_node_id) + " is the winner according to Node " +
    #              str(node_object.node_id))

    # check whether I am the winner
    if node_object.node_id == max_vote_having_entry_node_id:
        # update my flags
        node_object.node_flag = "leader"
        node_object.current_leader_id = node_object.node_id
        # broadcast the message
        leaders_broadcast = LeadersBroadcast(
            new_rsv_value=node_object.get_my_rsv(),
            node_list=node_object.get_active_node_ids()
        )
        communicator.broadcast_message(message=leaders_broadcast, sender_id=node_object.node_id)
        # election finish flag
        node_object.is_election_started = False

        logging.info("Node " + str(node_object.node_id) + " is elected as the leader")
    else:
        while not node_object.i_know_leader_is_elected and node_object.is_election_started:
            time.sleep(0.01)
        # election finish flag
        node_object.is_election_started = False


def start_election(node_object, communicator: Communicator):
    # stop drone movement and hover inplace
    node_object.dronekit_object.hover_the_drone()

    # ============================= <First Round of Election> =============================
    # start first round timer
    node_object.election_first_Round_starts_at = time.time()
    # start first round tacks
    election_round_one(node_object=node_object, communicator=communicator)
    # wait until first round ends
    while ((node_object.election_first_Round_starts_at * 1000) +
           system_parameters.time_taken_for_election_end) > (time.time() * 1000):
        time.sleep(0.01)

    # check for heartbeat receive after begin the election if so, hold the election
    if ((node_object.last_heartbeat_received_at * 1000) +
        system_parameters.leader_failure_detect_from_last_heartbeat) \
            > (time.time() * 1000):
        # stop election
        node_object.is_election_started = False
        logging.info(
            "Election is stopped(after round 1) for node " + str(node_object.node_id) + " due to heartbeat receive.")
        return

    logging.info("First round of election is finished for node " + str(node_object.node_id) + ".")
    # ============================= <Second Round of Election> =============================
    # start second round timer
    node_object.election_second_Round_starts_at = time.time()
    # start second round tacks
    election_round_two(node_object=node_object, communicator=communicator)
    # wait until second round ends
    while ((node_object.election_second_Round_starts_at * 1000) +
           system_parameters.time_taken_for_election_end) > (time.time() * 1000):
        time.sleep(0.01)

    # check for heartbeat receive after begin the election if so, hold the election
    if ((node_object.last_heartbeat_received_at * 1000) +
        system_parameters.leader_failure_detect_from_last_heartbeat) \
            > (time.time() * 1000):
        # stop election
        node_object.is_election_started = False
        logging.info(
            "Election is stopped(after round 2) for node " + str(node_object.node_id) + " due to heartbeat receive.")
        return

    logging.info("Second round of election is finished for node " + str(node_object.node_id) + ".")
    # ============================= <Third Round of Election> =============================
    # start third round timer
    node_object.election_third_Round_starts_at = time.time()
    # start third round tacks
    election_round_three(node_object=node_object, communicator=communicator)

    print("Node " + str(node_object.node_id) + " is know about new leader. It is " + str(
        node_object.current_leader_id) + ". \n")

    # wait until third round ends
    while ((node_object.election_third_Round_starts_at * 1000) +
           system_parameters.time_taken_for_election_end) > (time.time() * 1000):
        time.sleep(0.01)

    # check for heartbeat receive after begin the election if so, hold the election
    if ((node_object.last_heartbeat_received_at * 1000) +
        system_parameters.leader_failure_detect_from_last_heartbeat) \
            > (time.time() * 1000):
        # stop election
        node_object.is_election_started = False
        logging.info("Election is stopped(after round 3) for node " + str(
            node_object.node_id) + " due to heartbeat receive.")
        return

    logging.info("Third round of election is finished for node " + str(node_object.node_id) + ".")
    if not node_object.i_know_leader_is_elected:
        print(
            "Some issue with the election, need to start again. By node with id: " + str(node_object.node_id) + ". \n")
        start_election(node_object=node_object, communicator=communicator)
    else:
        print("Election is done. By node with id: " + str(node_object.node_id) + ". \n")

    logging.info("Election is finished for node " + str(node_object.node_id) + ".")
    # reset heartbeat timer
    node_object.last_heartbeat_received_at = time.time()
