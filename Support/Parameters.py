from dronekit import LocationGlobalRelative


class Parameters:
    vehicle_takeoff_altitude = 20
    path_points = [
        LocationGlobalRelative(-35.35977870, 149.16643020, 10),
        LocationGlobalRelative(-35.36035610, 149.16923050, 20),
        LocationGlobalRelative(-35.36137990, 149.16874770, 30),
        LocationGlobalRelative(-35.36163360, 149.16721340, 40),
        LocationGlobalRelative(-35.36234230, 149.16826490, 50),
        LocationGlobalRelative(-35.36375090, 149.16795370, 60),
        LocationGlobalRelative(-35.36326210, 149.16523740, 70)
    ]

    drone_status_pool = ["NOT READY", "READY TO ARM", "ARMED", "TAKING OFF", "IN FLY", "LANDING", "LANDED", "DISARMED"]

    maximum_check_arms_per_vehicle = 3

    altitude_tolerance = 0.05  # 5%

    breaking_altitude_when_landing = 0.25

    inter_drone_distance = 0.000075

    vehicle_connection_strings = [
        "127.0.0.1:14011", "127.0.0.1:14021", "127.0.0.1:14031", "127.0.0.1:14041", "127.0.0.1:14051", "127.0.0.1:14061"
    ]

    swarm_base_location = [-35.36326210, 149.16523740]

    data_gathering_interval = 0.1  # seconds

    operational_interval = 0.05  # seconds

    radius_of_earth = 1  # just a parameter

    def get_vehicle_takeoff_altitude(self):
        return self.vehicle_takeoff_altitude

    def get_drone_status_pool(self):
        return self.drone_status_pool

    def get_path_points(self):
        return self.path_points

    def get_maximum_check_arms_per_vehicle(self):
        return self.maximum_check_arms_per_vehicle

    def get_altitude_tolerance(self):
        return self.altitude_tolerance

    def get_breaking_altitude_when_landing(self):
        return self.breaking_altitude_when_landing

    def get_inter_drone_distance(self):
        return self.inter_drone_distance

    def get_vehicle_connection_strings(self):
        return self.vehicle_connection_strings

    def get_swarm_base_location(self):
        return self.swarm_base_location

    def get_data_gathering_interval(self):
        return self.data_gathering_interval

    def get_operational_interval(self):
        return self.operational_interval

    def get_radius_of_earth(self):
        return self.radius_of_earth
