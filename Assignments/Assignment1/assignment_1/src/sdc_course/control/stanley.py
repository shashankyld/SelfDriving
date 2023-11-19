import math
import numpy as np
from sdc_course.utils.utility import *


class StanleyLateralController:
    """
    StanleyLateralController implements lateral control using the stanley controller.
    """

    def __init__(self, vehicle, K_cte):
        self._vehicle = vehicle
        self._k_cte = K_cte

    def run_step(self, waypoints):
        return self._stanley_control(waypoints, self._vehicle.get_transform())

    def _get_heading_error(self, waypoints, ind_nearest, vehicle_yaw):
        waypoint_delta_x = waypoints[ind_nearest + 1][0] - waypoints[ind_nearest][0]
        waypoint_delta_y = waypoints[ind_nearest + 1][1] - waypoints[ind_nearest][1]
        waypoint_heading = np.arctan2(waypoint_delta_y, waypoint_delta_x)
        heading_error = ((waypoint_heading - vehicle_yaw) + np.pi) % (2 * np.pi) - np.pi
        return heading_error

    def _get_steering_direction(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if cross_prod >= 0:
            return -1
        return 1

    def _stanley_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT STANLEY CONTROL HERE ###############
        #######################################################################
        return steering
