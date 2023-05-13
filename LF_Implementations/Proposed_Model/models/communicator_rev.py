from __future__ import annotations

import logging
import random
from threading import Lock

from LF_Implementations.Proposed_Model.models.message_rev import Message
from LF_Implementations.Proposed_Model.parameters.operational_parameters import OperationalParameters

message_queue_mutex = Lock()

system_parameters = OperationalParameters.get_instance()


class Communicator:
    """
    Use this class as a singleton for manage communication among node objects
    """
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    _message_queue: list = []
    message_counter: int = 0
    broadcast_counter: int = 0
    number_of_messages_in_queue = property(lambda self: len(self._message_queue))

    def __init__(self):
        self._message_queue = []
        self.message_counter = 0

    def send_message(self, message: Message):
        # acquire mutex lock before accessing the message queue
        message_queue_mutex.acquire()
        # add the message to the queue and increase the counter
        self._message_queue.append(message)
        self.message_counter += 1
        logging.info(
            f"Message {message.message_id} MT:{message.message_type} sent from {message.sender} to {message.receiver}"
            f" -> {message}"
        )
        # release the lock
        message_queue_mutex.release()

    def broadcast_message(self, sender_id: int, message):
        # acquire mutex lock before accessing the message queue
        Communicator.broadcast_counter += 1
        message_queue_mutex.acquire()
        logging.info(
            f"Message B.CastID: {Communicator.broadcast_counter}; MT:{message.__class__.__name__}; Sent "
            f"from {sender_id} to *(TO: {system_parameters.alive_node_tracker_num}); Message content -> {message}"
        )
        # go through the active nodes and create a message for each one including the sender
        for node_id, is_active in enumerate(system_parameters.alive_node_tracker):
            # create a new message and send it to the node which is active
            if is_active:
                # allow message loss
                if system_parameters.message_loss_probability > random.random():
                    logging.info(
                        f"Message B.CastID: {Communicator.broadcast_counter} sent from {sender_id} to {node_id + 1} "
                        f"with content {message} was lost."
                    )
                    continue
                else:
                    new_message = Message(sender=sender_id, receiver=node_id + 1, content=message)
                    self._message_queue.append(new_message)
                    self.message_counter += 1

        # release the lock
        message_queue_mutex.release()

    def get_message(self, node_id: int) -> Message | None:
        if self.number_of_messages_in_queue > 0:
            # acquire mutex lock before accessing the message queue
            message_queue_mutex.acquire()
            # go through the message queue and check for messages for node id
            for message in self._message_queue:
                if message.receiver == node_id:
                    # have a message for me
                    # remove it from the queue and release the lock and return the message
                    self._message_queue.remove(message)
                    message_queue_mutex.release()
                    return message
            # no message for me even though there are messages in the queue, release the lock and return None
            message_queue_mutex.release()
            return None
        else:
            # message queue is empty, return None
            return None
