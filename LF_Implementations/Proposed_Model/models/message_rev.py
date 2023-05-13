from __future__ import annotations

import time

from dronekit import LocationGlobalRelative

from Support.Coordinate import Coordinate

message_type = {
    "Heartbeat": 0,
    "ElectionReply": 1,
    "InformWinner": 2,
    "LeadersBroadcast": 3,
    "NodeGoingToFail": 4,
}


class Heartbeat:
    """
    This class is used to send heartbeat messages to the leader.
    """

    def __init__(self, rsv_value: float, leader_position, leader_state: LocationGlobalRelative, fail_flag: bool = False):
        """
        This method is used to initialize the heartbeat message.
        :param rsv_value: The resource stringth value of the agent.
        :param fail_flag: The flag to indicate if the agent is going to fail or not. Default is False(mean not).
        """
        self.rsv_value: float = rsv_value
        self.going_fail: bool = fail_flag
        self.created_at: time = time.time()
        self.leaders_position: Coordinate = leader_position
        self.leader_state: LocationGlobalRelative = leader_state

    def __str__(self) -> str:
        """
        This method is used to print the heartbeat message.
        """
        return "'Heartbeat': {'rsv_value': " + str(self.rsv_value) + \
            ", 'created_at': " + str(self.created_at) + "}"


class NodeGoingToFail:
    """
    This class is used to send the message to the peers saying the node is going to fail.
    """

    def __init__(self, rsv_value: float):
        """
        This method is used to initialize the node going to fail message.
        :param rsv_value: The resource stringth value of the agent.
        """
        self.rsv_value: float = rsv_value
        self.created_at: time = time.time()

    def __str__(self) -> str:
        """
        This method is used to print the node going to fail message.
        """
        return "'NodeGoingToFail': {'rsv_value': " + str(self.rsv_value) + \
            ", 'created_at': " + str(self.created_at) + "}"


class ElectionReply:
    """
    This class is used to send election reply messages to the peers.(first round election)
    """

    def __init__(self, rsv_value: float):
        """
        This method is used to initialize the election reply message.
        :param rsv_value: The resource stringth value of the agent.
        """
        self.rsv_value: float = rsv_value
        self.created_at: time = time.time()

    def __str__(self) -> str:
        """
        This method is used to print the election reply message.
        """
        return "'ElectionReply': {'rsv_value': " + str(self.rsv_value) + \
            ", 'created_at': " + str(self.created_at) + "}"


class InformWinner:
    def __init__(self, winner_id: int, winner_rsv: float, confidence: float):
        """
        This method is used to initialize the 'inform winner' message (second round election).
        """
        self.winner_id: int = winner_id
        self.winner_rsv: float = winner_rsv
        self.created_at: time = time.time()
        self.confidence_score: float = confidence

    def __str__(self) -> str:
        """
        This method is used to print the 'inform winner' message.
        """
        return "'InformWinner': {'winner_id': " + str(self.winner_id) + \
            ", 'winner_rsv': " + str(self.winner_rsv) + ", 'confidence_score': " + str(self.confidence_score) + "}"


class LeadersBroadcast:

    def __init__(self, new_rsv_value: float, node_list: list):
        """
        This method is used to initialize the leaders broadcast message(third round of election).
        :param new_rsv_value: The new resource stringth value of the leader.
        :param node_list: The list of the nodes in the swarm.
        """
        self.elected_time: time = time.time()
        self.new_rsv_value: float = new_rsv_value
        self.current_node_list: list = node_list

    def __str__(self) -> str:
        """
        This method is used to print the leaders broadcast message.
        """
        return "'LeadersBroadcast': {'elected_time': " + str(self.elected_time) + \
            ", 'new_rsv_value': " + str(self.new_rsv_value) + "}"


class Message:
    id_counter: int = 1

    def __init__(self, content, sender, receiver):
        self.message_id: int = Message.id_counter
        self.content = content
        self.message_type: int = message_type[content.__class__.__name__]
        self.sender: int = sender
        self.receiver: int = receiver
        self.timestamp: time = time.time()

        # update id_counter
        Message.id_counter += 1

    def __str__(self) -> str:
        return "{'Message': {" + str(self.content) + "}, 'Sender': " + str(self.sender) + ", 'Receiver': " + str(
            self.receiver) + ", 'Message_Type': " + str(self.message_type) + "}"
