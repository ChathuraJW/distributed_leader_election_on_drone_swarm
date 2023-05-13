import math

from Support.Parameters import Parameters

earth_radius = Parameters().get_radius_of_earth()


class Coordinate:
    def __init__(self, longitude: float, latitude: float, altitude: float):
        """
        This method is used to initialize the coordinate.
        :param longitude: longitude of the coordinate
        :param latitude: latitude of the coordinate
        :param altitude: altitude of the coordinate
        """
        self.z = None
        self.y = None
        self.x = None

        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude

        # update cartesian coordinate
        self.update_cartesian_coordinate()

    def update_cartesian_coordinate(self):
        # convert to cartesian coordinate
        self.x = earth_radius * math.cos(self.latitude) * math.cos(self.longitude)
        self.y = earth_radius * math.cos(self.latitude) * math.sin(self.longitude)
        self.z = earth_radius * math.sin(self.latitude)

    def get_longitude(self):
        """
        This method is used to get the longitude of the coordinate.
        :return: longitude of the coordinate
        """
        return self.longitude

    def get_latitude(self):
        """
        This method is used to get the latitude of the coordinate.
        :return: latitude of the coordinate
        """
        return self.latitude

    def get_altitude(self):
        """
        This method is used to get the altitude of the coordinate.
        :return: altitude of the coordinate
        """
        return self.altitude

    def get_coordinate(self):
        """
        This method is used to get the coordinate.
        :return: coordinate(latitude, longitude, altitude)
        """
        return self.longitude, self.latitude, self.altitude

    def set_longitude(self, longitude):
        """
        This method is used to set the longitude of the coordinate.
        :param longitude: new longitude of the coordinate
        """
        self.longitude = longitude
        # update cartesian coordinate
        self.update_cartesian_coordinate()

    def set_latitude(self, latitude):
        """
        This method is used to set the latitude of the coordinate.
        :param latitude: new latitude of the coordinate
        """
        self.latitude = latitude
        # update cartesian coordinate
        self.update_cartesian_coordinate()

    def set_altitude(self, altitude):
        """
        This method is used to set the altitude of the coordinate.
        :param altitude: new altitude of the coordinate
        """
        self.altitude = altitude
        # update cartesian coordinate
        self.update_cartesian_coordinate()

    def set_coordinate(self, longitude, latitude, altitude):
        """
        This method is used to set the coordinate.
        :param longitude: new longitude of the coordinate
        :param latitude: new latitude of the coordinate
        :param altitude: new altitude of the coordinate
        """
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude
        # update cartesian coordinate
        self.update_cartesian_coordinate()

    def print_coordinate(self):
        print("----")
        print("Longitude: ", self.longitude)
        print("Latitude: ", self.latitude)
        print("Altitude: ", self.altitude)
        print("----")

    def get_all_values(self):
        """
        This method is used to get all the values of the coordinate.
        """
        return self.longitude, self.latitude, self.altitude

    def get_cartesian_coordinate(self):
        """
        This method is used to get the cartesian coordinate.
        :return: cartesian coordinate(x, y, z)
        """
        return self.x, self.y, self.z

    def distance_to_given_coordinate(self, coordinate):
        """
        This method is used to calculate the distance between the current coordinate and the given coordinate.
        :param coordinate: given coordinate
        :return: distance between the current coordinate and the given coordinate
        """
        coordinate_x, coordinate_y, _ = coordinate.get_cartesian_coordinate()

        return math.sqrt((self.x - coordinate_x) ** 2 + (self.y - coordinate_y) ** 2)

    def angle_with_given_coordinate(self, coordinate):
        """
        This method is used to calculate the angle between the current coordinate and the given coordinate.
        :param coordinate: given coordinate
        :return: angle between the current coordinate and the given coordinate in degrees
        """
        coordinate_x, coordinate_y, _ = coordinate.get_cartesian_coordinate()

        return math.degrees(math.atan2(coordinate_y - self.y, coordinate_x - self.x))

    def create_new_coordinate_to_follow(self, l_val, psi_val):
        """
        This method will create new coordinates and return it according to the l and psi values
        :param l_val: l value
        :param psi_val: psi value
        :return: new coordinate object
        """
        x = self.x + l_val * math.sin(psi_val)
        y = self.y + l_val * math.cos(psi_val)
        z = earth_radius * math.sin(self.latitude)
        alt = self.altitude
        lat = math.asin(z / earth_radius)
        lon = math.atan2(x, y)
        return Coordinate(lon, lat, alt)

    def __str__(self):
        return "{'Coordinate': {'Longitude': " \
            + str(self.longitude) + ", 'Latitude': " + str(self.latitude) + ", 'Altitude': " + str(self.altitude) + "}}"
