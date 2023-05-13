import logging
import math
import time


def altitude_tracker(vehicle, vehicle_id, breaking_altitude, is_lower=False):
    """
    This function will be help to track the altitude of the vehicle
    Arguments:
        vehicle: DroneKit vehicle object
        vehicle_id: An integer represent vehicle unique id
        breaking_altitude: An float value, which use to compair with the current vehicle altitude
        is_lower: If this value is true means loop will break when the current vehicle altitude become lower to the breaking_altitude. Default values is False.
    """
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        # print("Vehicle " + str(vehicle_id) + " current altitude is " + str(current_alt))
        # check for vehicle reach to expected altitude
        if (current_alt < breaking_altitude) and is_lower:
            break
        elif (current_alt > breaking_altitude) and (not is_lower):
            break
        time.sleep(1)


# thread based approach for vehicle initial positioning
def init_formation_thread_worker(formation_points, drone_object, point):
    drone_object.arm_the_vehicle()
    drone_object.takeoff_the_drone(altitude_expected=5, tolerance=0.9)
    drone_object.move_to_a_point(point_coordinates=formation_points[point])

    # TODO removed the landing option; if it necessary uncomment the below line
    drone_object.land_the_vehicle()

    logging.info("Vehicle " + str(point) + " reached to the expected point")


def vector_adder(vector_1, vector_2, is_only_decompose=False):
    """
    This function will add two vectors and return the resulting vector
    Arguments:
        vector_1: size of the vector with the angle to the x-axis(+)
        vector_2: size of the vector with the angle to the x-axis(+)
        is_only_decompose: If this value is true, function will return the decomposed vector. Default value is False.
    Return: resulting vector with the angle to the x-axis(+)
    """
    # get division value towards
    division_towards_x_axis = vector_1[0] * math.cos(math.radians(vector_1[1])) + vector_2[0] * math.cos(
        math.radians(vector_2[1]))
    division_towards_y_axis = vector_1[0] * math.sin(math.radians(vector_1[1])) + vector_2[0] * math.sin(
        math.radians(vector_2[1]))

    if is_only_decompose:
        return [division_towards_x_axis, division_towards_y_axis]
    # get resulting vector angle
    resulting_vector_angle = 90 * (division_towards_y_axis / (division_towards_x_axis + division_towards_y_axis))

    value = division_towards_x_axis / math.cos(math.radians(resulting_vector_angle))

    return [value, round(resulting_vector_angle, 3)]


def transform_range(input_value, input_min, input_max, output_min, output_max):
    """
    This function will transform a value into one range to the other.
    Further, the result will be rounded to the 5th decimal place.
    Arguments:
        input_value: The value to be transformed
        input_min: The minimum value of the input range
        input_max: The maximum value of the input range
        output_min: The minimum value of the output range
        output_max: The maximum value of the output range
    """
    if input_value > input_max:
        input_value = input_max

    input_range = (input_max - input_min)
    output_range = (output_max - output_min)
    new_value = (((input_value - input_min) * output_range) / input_range) + output_min
    result = round(new_value, 5)

    # result value guarding
    result = result if result >= output_min else output_min
    result = result if result <= output_max else output_max

    return result
