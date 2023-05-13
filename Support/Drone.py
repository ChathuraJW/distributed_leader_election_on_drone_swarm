import time

from dronekit import connect, VehicleMode, Vehicle, LocationGlobalRelative
from pymavlink import mavutil

from Support.Parameters import Parameters
from Support.Utility import altitude_tracker

# define parameter object
operational_parameters = Parameters()


class Drone:
    def __init__(self, dronekit_object, vehicle_id):
        """
        The constructor function which call in object creation. Which create Drone object and change the vehicle mode
        to "GUIDED" by default.
        Parameters:
            dronekit_object: DroneKit vehicle object
            vehicle_id: Integer as the vehicle ID, (should be unique)
        """
        self.dronekit_object: Vehicle = None
        self.vehicle_id: int = 0
        self.vehicle_status: str = operational_parameters.get_drone_status_pool()[0]
        self.dronekit_object = dronekit_object
        self.vehicle_id = vehicle_id
        self.dronekit_object.mode = VehicleMode("GUIDED")

    def __del__(self):
        """
        The destructor function with call at object deletion. This will close the established connection with
        the vehicle.
        """
        self.dronekit_object.close()

    def _update_status(self, status_number=0):
        """
        This is a private function with change the status variable of the drone.
        Parameters:
            status_number: Integer value correspond to respective status
        """
        self.vehicle_status = operational_parameters.get_drone_status_pool()[status_number]

    # TODO wait to call
    def _connect_to_the_vehicle(self, connection_string):
        """
        A private function which establish connection with vehicle. This method wait to proceed to until connection
        established.
        Parameters:
            connection_string: String variable with IP address and PORT number
        """
        self.dronekit_object = connect(connection_string, baud=921600, wait_ready=True)

    def is_ready_to_arm(self, arm_tries=operational_parameters.get_maximum_check_arms_per_vehicle()):
        """
        This function will check all vehicles in the list are ready to arm; If not wait until all are ready. For each
        vehicle predefined number of try attempt have to succeed. If all vehicle unable to success then False will be
        returned. True if all succeed.
        Parameters:
            arm_tries: (Optional)Integer variable gives the max number of attempt to check the vehicle is ready to arm or not. Defalut value will be taken from the Prameter class.
        Returns:
            Boolean variable; True if all ready to arm; False if at least not ready to arm.
        """

        is_ready_to_arm = False
        # try defined time, is the vehicle ready to arm or not
        for _ in range(arm_tries):
            is_ready_to_arm = self.dronekit_object.is_armable
            time.sleep(0.5)

        if is_ready_to_arm:
            # change status variable
            self._update_status(status_number=1)
            return True
        else:
            return False

    def arm_the_vehicle(self):
        """
        This function will arm the vehicle. Prior to call this. Makesure to verify vehicle is ready to arm.
        """
        # apply arming if not armed already
        if not self.dronekit_object.armed:
            while not self.is_ready_to_arm():
                time.sleep(0.5)

            self.dronekit_object.armed = True

        # change status variable
        self._update_status(status_number=2)

    def close_the_connection(self):
        """
        This function will close the connection with the vehicle.
        """
        self.dronekit_object.close()

    def disarm_the_vehicle(self):
        """
        This function will disarm the vehicle.
        """
        self.dronekit_object.disarm()

        # change status variable
        self._update_status(status_number=7)

    def takeoff_the_drone(self, altitude_expected, tolerance=operational_parameters.get_altitude_tolerance()):
        """
        This function takeoff the vehicle to expected altitude with defined tolerance.
        Parameters:
            altitude_expected: Altitude in unit meters.
            tolerance: (Optional)Error tolerance of the latitude. Value should be between 0(0%) and 1(100%). Default value will be taken form the Parameter class.
        """

        self.dronekit_object.simple_takeoff(alt=altitude_expected)

        # change status variable
        self._update_status(status_number=3)

        # track the altitude
        altitude_tracker(
            vehicle=self.dronekit_object,
            vehicle_id=self.vehicle_id,
            breaking_altitude=altitude_expected * (1 - tolerance),
            is_lower=False
        )

        # change status variable
        self._update_status(status_number=4)

    def land_the_vehicle(self):
        """
        This function facilitate the vehicle's landing operation. After calling this vehicle start the landing process.
        Altitude will be tracked until vehicle complete the landing.
        """
        # change the vehicle mode
        self.dronekit_object.mode = VehicleMode("LAND")

        # change status variable
        self._update_status(status_number=5)

        # track the altitude
        altitude_tracker(
            vehicle=self.dronekit_object,
            vehicle_id=self.vehicle_id,
            breaking_altitude=operational_parameters.get_breaking_altitude_when_landing(),
            is_lower=True
        )

        # change vehicle mode back to GUIDED
        self.dronekit_object.mode = VehicleMode("GUIDED")

        # change status variable
        self._update_status(status_number=6)

    def move_to_a_point(self, point_coordinates: LocationGlobalRelative, sleep_time=5, ground_speed=10, tracking_need=True):
        """
        This function will move the vehicle to the given point.
        Parameters:
            point_coordinates: DroneKit LocationGlobalRelative type location object
            sleep_time: (Optional)Time in seconds to wait after reaching the point. Default value is 5 seconds.
            ground_speed: (Optional)Ground speed in unit meters per second. Default value is 10 meters per second.
        """
        self.dronekit_object.simple_goto(point_coordinates, groundspeed=ground_speed)
        # wait few time
        time.sleep(sleep_time)

        # track reached to the point or not
        if tracking_need:
            while self.dronekit_object.groundspeed > 1:
                time.sleep(0.5)

    def hover_the_drone(self):
        """
        This function will hover the drone in place until new movement set by the program.
        First we take the current location of the drone and then give move instruction to that location. It will stop
        the movement of the drone and hover in place.
        """
        current_location = self.dronekit_object.location.global_relative_frame
        self.dronekit_object.simple_goto(current_location)

    def is_armed(self):
        """
        This function return the arm status of the drone.
        Returns:
            Boolean variable; True if drone is armed otherwise false.
        """
        return self.dronekit_object.armed

    def get_current_location(self):
        """
        This function return the current location of the drone.
        Returns:
            Dictionary variable with altitude(alt), latitude(lat) and longitude(lon).
        """
        current_location = self.dronekit_object.location.global_frame
        return {"alt": current_location.alt, "lat": current_location.lat, "lon": current_location.lon}

    def get_attitude(self):
        """
        This function return the current attitude of the drone.
        Returns:
            Dictionary variable with roll, pitch and yaw.
        """
        current_attitude = self.dronekit_object.attitude
        return {"roll": current_attitude.roll, "pitch": current_attitude.pitch, "yaw": current_attitude.yaw}

    def get_battery(self):
        """
        This function return the current battery status of the drone.
        Returns:
            Dictionary variable with voltage, current and level.
        """
        current_battery = self.dronekit_object.battery
        return {"voltage": current_battery.voltage, "current": current_battery.current, "level": current_battery.level}

    def move_drone_using_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.dronekit_object.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.dronekit_object.send_mavlink(msg)


