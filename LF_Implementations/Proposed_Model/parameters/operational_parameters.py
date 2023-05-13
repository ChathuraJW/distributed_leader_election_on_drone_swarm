import json

from dronekit import LocationGlobalRelative


class OperationalParameters:
    _heartbeat_interval = 4000  # 2000
    # TODO make this parameter a dynamic, considering the speed of the drone
    """in milliseconds, represents how frequent leader send heartbeats to followers"""

    _leader_failure_detection_factor = 2.0
    """represents how many times heartbeat interval should be waited to detect leader failure"""

    _leader_failure_detect_from_last_heartbeat = _heartbeat_interval * _leader_failure_detection_factor
    """this parameter represent how long should it take (in ms) to recognize the leader failure"""

    _time_taken_for_election_end_factor = 1.0
    """n times heartbeat interval is the value, start when node get into know the failure of the leader"""

    _time_taken_for_election_end = _heartbeat_interval * _time_taken_for_election_end_factor
    """this parameter represent how long should it take (in ms) to end the election"""

    _time_taken_for_election_confirmation_end = 1.5
    """n times heartbeat interval is the value, start when node get into know the failure of the leader"""

    _time_taken_for_election_confirmation = _heartbeat_interval * _time_taken_for_election_confirmation_end
    """this parameter represent how long should it take (in ms) to end the election confirmation(second round)"""

    _node_failing_probability = 0.25
    """
    represents the probability of a node to fail, this is used only to simulate the failure of a node. Other than that, 
    this is not used in real implementations.
    """

    _rsv_threshold_to_define_leader_failure = 30
    _rsv_threshold_to_define_follower_failure = 40
    """
    this value will use when a node get a heartbeat and to variety that the leader is going tp fail or not. And to 
    create the going to fail message when leader has the lower rsv value than this threshold. 
    """

    _number_of_nodes = 5
    """
    this parameter defined the number nodes initially at the drone swarm.
    """

    _initial_leader: int = 1
    """
    this parameter defined the initial leader of the drone swarm.
    """

    _alive_node_tracker = [True] * _number_of_nodes
    """
    this parameter is used to track the alive nodes in the drone swarm.
    """
    _leader_rsv_depreciation_accepted_rate = 0.2
    """
    this parameter is used to define the rate of depreciation of the leader rsv value allowed inorder to terminate 
    ongoing election process.
    """

    _allow_follower_informed_failure = True
    _allow_follower_immediate_failure = True
    """
    this parameter is used to define if the follower failure is allowed or not.
    """

    _vehicle_takeoff_altitude = 5

    _path_points_for_leader = []
    """
    path points for the leader drone.
    """

    _assigned_priority_for_drone = [50, 40, 30, 58, 79]
    """
    priority for each node, 0th element for drone with node id 1 and so on.
    """

    _payload_for_each_drone = [0.05, 0.03, 0.03, 0.09, 0.06]
    """
    payload value for each drone, values represents in unit KG, 0th element for drone with node_id 1 and so on.
    """

    _rsv_scrape_interval = 2000
    """
    this parameter represents the interval in which the rsv value of the individual nodes to be scraped.
    """

    _max_attitude_threshold = 180
    """
    this parameter represents the maximum attitude threshold of the drone in degrees for roll, pitch and yaw parameters.
    """

    _map_rounds = 3
    """
    this parameter represents the number of rounds the drone should fly in the map.
    """

    _message_loss_probability = 0.05
    """
    this parameter represents the probability of message loss.
    -1.0 means no message loss or disable message loss
    """

    # make the class singleton
    __instance = None

    @classmethod
    def get_instance(cls):
        if cls.__instance is None:
            cls.__instance = OperationalParameters()

        # load path list values, read json file and fill the variable
        with open("LF_Implementations/Proposed_Model/parameters/nav_points.json", "r") as file:
            data = json.load(file)
        output_list = []
        for entry in data["long_path_1"]:
            output_list.append(eval(entry))
        cls._path_points_for_leader = output_list

        return cls.__instance

    def __init__(self):
        if OperationalParameters.__instance is not None:
            raise Exception("Only one instance of Singleton is allowed")

    def get_active_node_list(self):
        active_node_list = []
        for i, status in enumerate(self.alive_node_tracker):
            if status:
                active_node_list.append(i + 1)
        return active_node_list

    # create getters for all the parameters
    @property
    def heartbeat_interval(self):
        return self._heartbeat_interval

    @property
    def leader_failure_detection_factor(self):
        return self._leader_failure_detection_factor

    @property
    def leader_failure_detect_from_last_heartbeat(self):
        return self._leader_failure_detect_from_last_heartbeat

    @property
    def time_taken_for_election_end_factor(self):
        return self._time_taken_for_election_end_factor

    @property
    def time_taken_for_election_end(self):
        return self._time_taken_for_election_end

    @property
    def time_taken_for_election_confirmation_end(self):
        return self._time_taken_for_election_confirmation_end

    @property
    def time_taken_for_election_confirmation(self):
        return self._time_taken_for_election_confirmation

    @property
    def node_failing_probability(self):
        return self._node_failing_probability

    @property
    def rsv_threshold_to_define_leader_failure(self):
        return self._rsv_threshold_to_define_leader_failure

    @property
    def rsv_threshold_to_define_follower_failure(self):
        return self._rsv_threshold_to_define_follower_failure

    @property
    def number_of_nodes(self):
        return self._number_of_nodes

    @property
    def initial_leader(self) -> int:
        return self._initial_leader

    @property
    def alive_node_tracker(self):
        return self._alive_node_tracker

    @property
    def alive_node_tracker_num(self):
        return_vec = []
        for i, bv in enumerate(self._alive_node_tracker):
            if bv:
                return_vec.append(i + 1)
        return return_vec

    @property
    def leader_rsv_depreciation_accepted_rate(self):
        return self._leader_rsv_depreciation_accepted_rate

    @property
    def vehicle_takeoff_altitude(self):
        return self._vehicle_takeoff_altitude

    @property
    def path_points_for_leader(self):
        return self._path_points_for_leader

    def get_path_points_for_leader_list_length(self):
        return len(self._path_points_for_leader)

    @property
    def assigned_priority_for_drone(self):
        return self._assigned_priority_for_drone

    @property
    def payload_for_each_drone(self):
        return self._payload_for_each_drone

    @property
    def rsv_scrape_interval(self):
        return self._rsv_scrape_interval

    @property
    def max_attitude_threshold(self):
        return self._max_attitude_threshold

    @property
    def map_rounds(self):
        return self._map_rounds

    @property
    def allow_follower_immediate_failure(self):
        return self._allow_follower_immediate_failure

    @property
    def allow_follower_informed_failure(self):
        return self._allow_follower_informed_failure

    @property
    def message_loss_probability(self):
        return self._message_loss_probability
