from collections import deque
import math
import numpy as np
from sdc_course.utils.utility import *


class PIDLongitudinalController:
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """

        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10)
        self._prev_velocity = 0

    def run_step(self, target_velocity_ms, debug=False):
        """
        Execute one step of longitudinal control to reach a given target velocity.

        :param target_velocity_ms: target velocity in m/s
        :param debug: boolean for debugging
        :return: throttle control
        """
        current_velocity_ms = get_velocity_ms(self._vehicle)

        if debug:
            print("Current velocity = {}".format(current_velocity_ms))

        return self._pid_control(target_velocity_ms, current_velocity_ms)

    def _pid_control(self, target_velocity_ms, current_velocity_ms):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

        :param target_velocity_ms:  target velocity in m/s
        :param current_velocity_ms: current velocity of the vehicle in m/s
        :return: throttle/brake control
        """
        acceleration = 0.0
        
        #######################################################################
        ################## TODO: IMPLEMENT LONGITUDINAL PID CONTROL HERE ######
        #######################################################################
         
        p_term = self._k_p * (target_velocity_ms - current_velocity_ms) 
        d_term = self._k_d * (current_velocity_ms - self._prev_velocity)

        acceleration += p_term + d_term
        self._prev_velocity = current_velocity_ms
        return acceleration


class PIDLateralController:
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self.prev_steering_error = 0
        self._error_buffer = deque(maxlen=10)

    def run_step(self, waypoints):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

        :param waypoint: target waypoint
        :return: steering control
        """
        return self._pid_control(waypoints, self._vehicle.get_transform())

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

    def _get_steering_magnitude(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        return abs(cross_prod)

    def _pid_control(self, waypoints, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoints: local waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        ######################################################################
        ################## TODO: IMPLEMENT LATERAL PID CONTROL HERE ###########
        #######################################################################
        ''' 
        vehicle_transform:  
        
        Transform
        (
            Location(x=-36.790024, y=206.752975, z=-0.005528), 
            Rotation(pitch=0.103115, yaw=0.030729, roll=-0.001129) # In angles
        )
        waypoints:  
        [
            [-36.493443, 206.744247, 35.801], 
            [-34.461002, 206.742599, 37.993], 
            [-32.19743, 206.740662, 42.876], 
            [-30.167361962128414, 206.73901959269804, 45.93325080185188], 
            [-28.128602267842417, 206.73720499546002, 48.50106485924268], 
            [-26.12333361373597, 206.7355161672962, 50.77550338912468], 
            [-24.09943548741853, 206.73386214436925, 52.91540689863742], 
            [-21.917484, 206.732086, 55.099], 
            [-19.861917131168074, 206.73026520589548, 55.621883630637065], 
            [-17.826444695687304, 206.72851201714678, 58.07357981998717], [-15.560539, 206.726639, 60.324]]

        '''
        # print("Vehicle Transform Location: ", self._vehicle.get_transform().location)
        # print("waypoints: ", type(waypoints), waypoints)
        # print("vehicle_transform: ", type(vehicle_transform),  vehicle_transform)
        vehicle_pose = [vehicle_transform.location.x, vehicle_transform.location.y]
        waypoint0 = waypoints[0][:2]
        waypoint1 = waypoints[1][:2]
        tangent_direction = np.array(waypoint1) - np.array(waypoint0)
        v1 = np.array(waypoint0) - np.array(vehicle_pose) # vehicle to waypoint
        yaw = math.radians(vehicle_transform.rotation.yaw)
        v2 = [np.cos(yaw), np.sin(yaw)] #yaw_vec
        steering_direction = self._get_steering_direction(v1, v2)
        steering_error = self._get_steering_magnitude(v1, v2)
        # print("v1", v1)
        # print("yaw", yaw)
        # print("yaw_vec", v2)
        # print("steering_direction", steering_direction)
        p_term = self._k_p * steering_direction * steering_error
        d_term = -1 * self._k_d * (steering_error - self.prev_steering_error)
        print("steering error", steering_error)
        print("prev_steering error", self.prev_steering_error)
        print("p_term", p_term)
        print("d_term", d_term)
        steering += p_term + d_term
        self.prev_steering_error = steering_error
        return steering
