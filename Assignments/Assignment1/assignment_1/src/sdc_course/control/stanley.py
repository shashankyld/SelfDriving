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
        # delta = psi + tan_inv (K * e_cte / v)
        # psi = yaw - tangent_direction
        # Front wheel pos = vehicle pos + L * [cos(yaw), sin(yaw)]
        # front_wheel_pos = np.array([vehicle_transform.location.x, vehicle_transform.location.y]) + self._vehicle.L * np.array([np.cos(vehicle_transform.rotation.yaw * math.pi / 180), np.sin(vehicle_transform.rotation.yaw * math.pi / 180)])
        # Find the nearest waypoint from front wheel - but approximate by nearest waypoint from vehicle position
        nearest_waypoint, ind_nearest = get_nearest_waypoint(self._vehicle, waypoints)
        # Get the tangent direction from nearest waypoint to next waypoint
        # tangent_direction = math.atan2(waypoints[ind_nearest + 1][1] - waypoints[ind_nearest][1], waypoints[ind_nearest + 1][0] - waypoints[ind_nearest][0])
        # Get the heading error
        heading_error = self._get_heading_error(waypoints, ind_nearest, vehicle_transform.rotation.yaw * math.pi / 180)
        # Get the cross track error
        cross_track_error = compute_distance_from_front_wheel_to_waypoint(self._vehicle, nearest_waypoint)
        # Get the psi
        # psi = vehicle_transform.rotation.yaw * math.pi / 180 - tangent_direction
        # Get the delta
        speed = np.sqrt(self._vehicle.get_velocity().x ** 2 + self._vehicle.get_velocity().y ** 2)
        if speed != 0:
            delta = 0
            delta = heading_error
            delta += - np.arctan(self._k_cte * cross_track_error / speed) * self._get_steering_direction([np.cos(vehicle_transform.rotation.yaw * math.pi / 180), np.sin(vehicle_transform.rotation.yaw * math.pi / 180)], [waypoints[ind_nearest + 1][0] - waypoints[ind_nearest][0], waypoints[ind_nearest + 1][1] - waypoints[ind_nearest][1]])
        else:
            delta = 0.0  # or any other appropriate value when speed is zero


        steering = delta

        

        return steering

def compute_distance_from_front_wheel_to_waypoint(vehicle, waypoint):
    """compute distance of vehicle to given waypoint."""
    vloc = vehicle.get_transform().location
    front_wheel = np.array([vloc.x, vloc.y]) + vehicle.L * np.array([np.cos(vehicle.get_transform().rotation.yaw * math.pi / 180), np.sin(vehicle.get_transform().rotation.yaw * math.pi / 180)])
    distance = math.sqrt((front_wheel[0] - waypoint[0]) ** 2 + (front_wheel[1] - waypoint[1]) ** 2)
    return distance
def nearest_waypoint(pose, trajectory):
    """get nearest waypoint for given vehicle.

    : param pose of object
    : param trajectory: waypoint list

    : return : tuple of nearest waypoint and index of nearest waypoint
    """
    distances = []
    for wp in trajectory:
        dist = compute_distance_to_waypoint(pose, wp)
        distances.append(dist)
    ind_nearest = distances.index(min(distances))
    nearest_waypoint = trajectory[ind_nearest]

    return nearest_waypoint, ind_nearest
    