import time

from LF_Implementations.Proposed_Model.models.communicator_rev import Communicator
from LF_Implementations.Proposed_Model.models.message_rev import Message, Heartbeat, ElectionReply, InformWinner, \
    LeadersBroadcast, NodeGoingToFail
from LF_Implementations.Proposed_Model.parameters.operational_parameters import OperationalParameters

system_parameters = OperationalParameters.get_instance()


def process_message(self, message: Message, communicator: Communicator):
    if isinstance(message.content, Heartbeat):
        # if leader election is in the middle and leader recovered
        if self.is_election_started:
            # check leader's previous rsv and the new rsv value, the check for deprecations
            if message.content.rsv_value > self.leader_rsv * (
                    1 - system_parameters.leader_rsv_depreciation_accepted_rate):
                # leader's status is good, terminate the ongoing election
                self.is_election_started = False
            else:
                # ignore the heartbeat message
                return

        # update parameters for current leader location, time received and rsv
        self.current_leader_position = message.content.leaders_position
        self.last_heartbeat_received_at = time.time()
        self.leader_rsv = message.content.rsv_value
        self.leader_goal_state = message.content.leader_state

        # TODO add going to fail part

    elif isinstance(message.content, ElectionReply):
        # process the election reply accordingly
        self.rsv_vector[message.sender - 1] = {
            "node_id": message.sender,
            "rsv": message.content.rsv_value,
            "calculated_at": message.content.created_at
        }

    elif isinstance(message.content, InformWinner):
        # process the "inform winner" message accordingly
        # add the message details to vote vector
        self.vote_vector[message.sender - 1] = {
            "node_id": message.content.winner_id,
            "rsv": message.content.winner_rsv,
            "calculated_at": message.content.created_at,
            "confidence_score": message.content.confidence_score
        }

    elif isinstance(message.content, LeadersBroadcast):
        # process the leaders broadcast message accordingly
        # update my leader information according to broadcast
        self.current_leader_id = message.sender
        # clean rsv vector
        self.clean_rsv_vector()
        # clean vote vector
        self.clean_vote_vector()
        # make leader node vector position also a None
        self.rsv_vector[message.sender - 1] = None
        # make vote vector leader position also a None
        self.vote_vector[message.sender - 1] = None
        # change the flag
        self.i_know_leader_is_elected = True

    elif isinstance(message.content, NodeGoingToFail):
        # process the node going to fail message accordingly
        # make respective nodes rsv vector position a None
        self.rsv_vector[message.sender - 1] = None
        # make vote vector leader position also a None
        self.vote_vector[message.sender - 1] = None

    else:
        print("Invalid message type")
