from PID import PIDController
import numpy as np
from math import sqrt, degrees, atan

class WheelController():
    def __init__(self, space_center, rover, surface_reference_frame) -> None:
        self.running = True
        self.space_center = space_center
        self.rover = rover
        self.surface_ref = surface_reference_frame
        self.body_ref = self.rover.orbit.body.reference_frame
        self.target_pos = (0, 0, 0)
        self.target_dist = 0
        self.error_angle = 0
        self.speed = 0

        self.rover.control.brakes = False
        self.rover.control.wheel_throttle = 0
        self.rover.control.wheel_steering = 0

        self.safe_distance = 50
        self.speed_target = 5
        self.steering_target = 0

        # PID
        self.throttle_pid = PIDController()
        self.steering_pid = PIDController()

        self.throttle_pid.adjust_pid(0.5, 0.1, 0.5)

        self.throttle_pid.limit_output(-1, 1)
        self.steering_pid.limit_output(-1, 1)

    

    def start_loop(self):
        while self.running:
            target_pos = self.space_center.transform_position(self.target_pos, self.body_ref, self.surface_ref)[1:]
            target_pos = np.array(target_pos) * [1, -1]
            target_mag = sqrt(np.sum(target_pos**2)) # Distance - Mag
            self.target_dist = target_mag

            rover_dir = self.rover.direction(self.surface_ref)[1:]
            target_dir = target_pos/target_mag # Normilize pos to get direction

            error_dir = target_dir - rover_dir
            self.error_angle = degrees(atan(error_dir[0]/error_dir[1]))

            self.rover.control.wheel_throttle = self.throttle_pid.calc_pid(self.speed ,self.speed_target)
            self.rover.control.wheel_steering = self.steering_pid.calc_pid(self.error_angle, self.steering_target)

            if self.target_dist < self.safe_distance:
                self.rover.control.brakes = True
                self.rover.control.wheel_throttle = 0
                self.rover.control.wheel_steering = 0
                self.target_dist = 0
                self.error_angle = 0
                self.running = False

    def set_target_pos(self, pos):
        self.target_pos = self.space_center.transform_position(pos, self.surface_ref, self.body_ref)
