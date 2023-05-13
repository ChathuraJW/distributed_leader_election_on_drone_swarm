import math

import numpy as np
from dronekit import LocationGlobalRelative


# always first point should be the leaders positions

class Formation:
    formation_points = []
    formation_points_raw = []

    def get_leader_point_raw(self):
        return self.formation_points_raw[0]

    def get_followers_point_raw(self):
        return self.formation_points_raw[1:]

    def get_formation_points(self):
        """
        This function will return the formation_points list.
        """
        return self.formation_points

    def get_formation_points_raw(self):
        """
        This function will return the formation_points_raw list.
        """
        return self.formation_points_raw

    def make_triangle_formation_three_vehicles(self, base_coordinate, d=1.0):
        """
        This function create the formation of triangle using three vehicles.
        Above block shows the expected formation. As the input parameter point 1 coordinate should provide.

            *
        *       *

        :base_coordinate: This is a vector of two element for longitude and latitude respectively
        :d: Distance between vehicles
        """
        temp1 = d * math.sin(math.radians(60))
        temp2 = d * math.cos(math.radians(60))
        point_x = base_coordinate[0]
        point_y = base_coordinate[1]

        points = [
            [point_x, point_y],
            [point_x - temp1, point_y - temp2],
            [point_x + temp1, point_y - temp2]
        ]
        location_points = [
            LocationGlobalRelative(point_x, point_y, 10),
            LocationGlobalRelative(point_x - temp1, point_y - temp2, 10),
            LocationGlobalRelative(point_x + temp1, point_y - temp2, 10)
        ]
        self.formation_points_raw = points
        self.formation_points = location_points

    def make_line_formation_two_vehicle(self, base_coordinate, d=1.0):
        """
        This function create the formation of line using two vehicles.
        Above block shows the expected formation. As the input parameter point 1 coordinate should provide.

            *
            *

        :base_coordinate: This is a vector of two element for longitude and latitude respectively
        :d: Distance between vehicles
        """
        point_x = base_coordinate[0]
        point_y = base_coordinate[1]

        points = [
            [point_x, point_y],
            [point_x, point_y - d]
        ]
        location_points = [
            LocationGlobalRelative(point_x, point_y, 10),
            LocationGlobalRelative(point_x, point_y - d, 10)
        ]
        self.formation_points_raw = points
        self.formation_points = location_points

    def make_arrow_formation_six_vehicles(self, base_coordinate, d=1.0):
        """
        This function create the formation of cross using six vehicles.
        Above block shows the expected formation. As the input parameter point 1 coordinate should provide.

            *
        *   *   *
            *
            *

        Arguments:
              base_coordinate: This is a vector of two element for longitude and latitude respectively
              d: Distance between vehicles
        """
        template = np.array([
            [None, lambda x, y: (x, y), None],
            [lambda x, y: (x - d, y - d), lambda x, y: (x, y - d), lambda x, y: (x + d, y - d)],
            [None, lambda x, y: (x, y - d * 2), None],
            [None, lambda x, y: (x, y - d * 3), None]
        ])
        for row in range(template.shape[0]):
            for column in range(template.shape[1]):
                if template[row][column]:
                    # get alt and lnt values
                    points = template[row][column](base_coordinate[0], base_coordinate[1])
                    # get raw formation point
                    self.formation_points_raw.append((points[0], points[1]))
                    # spend dronekit LocationGlobalRelative object to the formation_points list
                    self.formation_points.append(LocationGlobalRelative(points[0], points[1], 10))
