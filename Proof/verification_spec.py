import random
import threading
import time

TOTAL_NODE_COUNT = 5
RSV_VECTOR = []
WAIT_TIME = 2000
LFR = 0.3
MESSAGE_LOSS_PROBABILITY = 0.0

# fill RSV vector with unique values
# in real algorithm this vector will be filled with real values
# calculated by each node, before use it in the algorithm
for j in range(TOTAL_NODE_COUNT):
    val = random.randint(0, 100)
    if val not in RSV_VECTOR:
        RSV_VECTOR.append(val)


class DroneModel:
    def __init__(self, node_id, leader_id):

        self.all_nodes = None
        self.node_id = node_id
        self.my_rsv = 0.0

        # hardcode the initial leader
        self.leader_id = leader_id

        self.last_hb_time = time.time()
        self.is_leader_alive = True

        # initialze rsv and vote vectors
        self.rsv_vector = [[0, 0]] * TOTAL_NODE_COUNT
        self.vote_vector = [[0, 0]] * TOTAL_NODE_COUNT

    def set_node_obj_list(self, node_objects):
        self.all_nodes = node_objects

    def communicate(self, message):
        if message[0] == 0:  # heartbeat message [0, leader_id, rsv_val]
            self.is_leader_alive = True
            self.leader_id = message[1]
            self.last_hb_time = time.time()

        if message[0] == 1:  # election reply [1, node_id, rsv_val, cal_time]
            self.rsv_vector[message[1] - 1] = [message[2], message[3]]

        if message[0] == 2:  # inform winner [2, node_id, confidence_score]
            self.vote_vector[message[1] - 1] = [message[1], message[2]]

        if message[0] == 3:  # leader's broadcast [3, leader_id]
            self.is_leader_alive = True
            self.leader_id = message[1]

    def broadcast(self, message):
        for node in self.all_nodes:
            # simiulate message loss
            if random.random() > MESSAGE_LOSS_PROBABILITY:
                node.communicate(message)

    def leader_work(self):
        while True:
            if random.random() > LFR:
                # leader alive, make broadcast
                # get RSV from pre-defined vector
                self.my_rsv = RSV_VECTOR[self.node_id - 1]
                hb_message = [0, self.node_id, self.my_rsv]
                self.broadcast(hb_message)
                time.sleep(WAIT_TIME / 2000)
            else:
                # leader failed
                print("Leader failed")
                exit(10)

    def follower_work(self):
        while True:
            # print(self.last_hb_time)
            if self.last_hb_time + WAIT_TIME < time.time():
                # normal operation
                print("Node with ID: " + str(self.node_id) +
                      " move according to leader, with node ID: " + str(self.leader_id) + ".\n")
            else:
                # leader failed, do election
                # ------------------------ Round 1 ------------------------
                timer_r1 = time.time()
                self.my_rsv = random.randint(0, 100)
                er_message = [1, self.node_id, self.my_rsv, time.time()]
                self.broadcast(er_message)
                # wait until round 1 finish
                while timer_r1 + WAIT_TIME < time.time():
                    time.sleep(0.001)

                # ------------------------ Round 2 ------------------------
                timer_r2 = time.time()
                sorted_vector = sorted(
                    enumerate(self.rsv_vector),
                    key=lambda x: (x[1][0], -x[1][1])
                )
                my_selection_node_id = sorted_vector[-1][0] + 1
                confidence_value = len(self.rsv_vector) / \
                    (TOTAL_NODE_COUNT - 1)
                info_message = [2, my_selection_node_id, confidence_value]
                self.broadcast(info_message)
                while timer_r2 + WAIT_TIME < time.time():
                    time.sleep(0.001)

                # ------------------------ Round 3 ------------------------
                timer_r3 = time.time()

                vote_counter = [0] * TOTAL_NODE_COUNT
                for vote in self.vote_vector:
                    vote_counter[vote[0]] = vote_counter[vote[0]] + vote[1]

                sorted_indices = [x[0] for x in sorted(
                    enumerate(vote_counter),
                    key=lambda x: -x[1]
                )]

                selected_new_leader_id = 0
                for i, ind in enumerate(sorted_indices):
                    if ind == 0:
                        selected_new_leader_id = i + 1
                        break

                if selected_new_leader_id == self.node_id:
                    # I am the leader
                    lb_message = [3, self.node_id]
                    self.broadcast(lb_message)

                while timer_r3 + WAIT_TIME < time.time():
                    time.sleep(0.001)

                print("I am node with ID: " + str(self.node_id) + ". And I know node with ID: " +
                      str(self.leader_id) + " is selected as the leader.\n")
                break

    def do_work(self):
        if self.node_id == self.leader_id:
            # I am the leader, do leader's work
            threading.Thread(target=self.leader_work).start()
        else:
            # I am a follower, do follower's work
            threading.Thread(target=self.follower_work).start()


def main():
    all_nodes = [DroneModel(node_id=i + 1, leader_id=1)
                 for i in range(TOTAL_NODE_COUNT)]
    for i in range(TOTAL_NODE_COUNT):
        all_nodes[i].set_node_obj_list(all_nodes)
        all_nodes[i].do_work()


if __name__ == '__main__':
    main()
