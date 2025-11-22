#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        self.vars.create_var("idx", 0)
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
            self.vars.idx = min_idx
        else:
            desired_speed = self._waypoints[-1][2]
            self.vars.idx = len(self._waypoints) -1

        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        idx = self.vars.idx
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time

            P_throttle = 1
            I_throttle = 0.1
            D_throttle = 0.5

            self.vars.create_var("_previous_timestamp", 0)
            self.vars.create_var("_previous_I", 0)
            self.vars.create_var("_previous_err", 0)
           
            length_waypoints = len(waypoints)

            if(self.vars.idx < length_waypoints -1):
                
                err_goal = np.hypot(waypoints[-1][0]-x, waypoints[-1][1]-y)

                err_speed = v_desired - v

                P_output = P_throttle * err_speed
                I_output = self.vars._previous_I + I_throttle * err_speed * (t - self.vars._previous_timestamp)
                
                if ((t - self.vars._previous_timestamp) >0.001 ):
                    D_output = D_throttle * (err_speed-self.vars._previous_err) / (t - self.vars._previous_timestamp)
                else:
                    D_output = 0
                
                throttle_output = P_output + I_output + D_output

                # Optional brake
                if err_speed < -0.5:  # If the car is significantly faster than desired
                    brake_output = np.clip(-throttle_output, 0, 1)
                    throttle_output = 0
                else:
                    brake_output = 0

                self.vars._previous_timestamp = t
                self.vars._previous_I = I_output
                self.vars._previous_err = err_speed

                ######################################################
                ######################################################
                # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
                ######################################################
                ######################################################
                """
                    Implement a lateral controller here. Remember that you can
                    access the persistent variables declared above here. For
                    example, can treat self.vars.v_previous like a "global variable".
                """


                # Change the steer output with the lateral controller. 
                self.vars.create_var("prev_steer", 0)
                # Pure Pursuit Controller
                L = 1.5  # 차량의 휠베이스
                lookahead_distance = 5.0 + 0.5 * v  # 동적 lookahead distance
                

                # 2. 가장 가까운 Waypoint와 그 다음 Waypoint 가져오기
                # 마지막 Waypoint일 경우 idx+1을 조정
                if idx < len(waypoints) - 1:
                    next_idx = idx + 1
                else:
                    next_idx = idx

                current_wp = waypoints[idx]
                next_wp = waypoints[next_idx]

                # 3. Waypoint 사이 거리 계산
                wp_dx = next_wp[0] - current_wp[0]
                wp_dy = next_wp[1] - current_wp[1]
                wp_distance = np.hypot(wp_dx, wp_dy)

                # 4. 보간 비율 계산
                if wp_distance > 0:
                    ratio = lookahead_distance / wp_distance
                    if ratio > 1.0:
                        ratio = 1.0  # 보간 거리 제한 (다음 Waypoint를 넘어가지 않도록)
                else:
                    ratio = 0.0  # Waypoints가 중첩된 경우 대비

                # 5. 보간을 통해 목표 포인트 계산
                goal_x = current_wp[0] + ratio * wp_dx
                goal_y = current_wp[1] + ratio * wp_dy

                dx = goal_x - x
                dy = goal_y - y

                alpha = np.arctan2(dy,dx) - yaw
                alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
               

                steer_output = np.arctan2(2*L*np.sin(alpha), lookahead_distance)
                max_steer_change = np.deg2rad(10)  # 한 번의 스텝에서 최대 5도 변경
                steer_output = np.clip(steer_output, self.vars.prev_steer - max_steer_change, self.vars.prev_steer + max_steer_change)
                self.vars.prev_steer = steer_output  # 이전 값 업데이트
                
                if err_goal < 2.0:
                    print("Vehicle has reached the goal.")
                    # Stop the vehicle
                    self.set_throttle(0.0)
                    self.set_brake(1.0)
                    self.set_steer(0.0)

            else:
                brake_output = 1.0
                throttle_output = 0

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step

