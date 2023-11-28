import math
import numpy as np
from sdc_course.utils.utility import *


class PurePursuitLateralController:
    """
    PurePursuitLateralController implements lateral control using the pure pursuit controller.
    """

    def __init__(self, vehicle, L, ld, K_pp):
        self._vehicle = vehicle
        self._L = L # Length of the car
        self._ld = ld # Lookahead distance
        self._k_pp = K_pp # Pure pursuit gain 

    def run_step(self, waypoints):
        return self._pure_pursuit_control(waypoints, self._vehicle.get_transform())

    def _get_goal_waypoint_index(self, vehicle, waypoints, lookahead_dist):
        for i in range(len(waypoints)):
            dist = compute_distance_to_waypoint(vehicle, waypoints[i])
            if dist >= lookahead_dist:
                return max(0, i)
        return len(waypoints) - 1

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

    def _pure_pursuit_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT PURE-PURSUIT CONTROL HERE ##########
        #######################################################################
        print("Vehicle Transform: ", vehicle_transform)
        # Get the vehicle position
        vehicle_position = np.array([vehicle_transform.location.x, vehicle_transform.location.y])
        print("Vehicle Position: ", vehicle_position)
        print("Waypoints: ", waypoints)
        # steering = tan_inv (2 * L * sin (alpha) / ld)
        # alpha = theta - yaw
        # theta = tan_inv (y2 - y1 / x2 - x1)
        # Target at lookahead distance from waypoints list 
        target_index = self._get_goal_waypoint_index(self._vehicle, waypoints, self._ld)
        theta = math.atan2(waypoints[target_index][1] - vehicle_position[1], waypoints[target_index][0] - vehicle_position[0])
        yaw = vehicle_transform.rotation.yaw * math.pi / 180
        alpha = theta - yaw
        steering = math.atan(2 * self._L * math.sin(alpha) / self._ld)
        
        
        return steering
