import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import sys

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, 
                    max_speed, min_speed, accel_limit, decel_limit,
                    max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.max_speed = max_speed
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.max_steer_angle = max_steer_angle
        self.throttle_pid = PID(4.0, 0.0, 0.0)
        self.brake_pid = PID(220.0, 20.0, 0.0)
        self.yaw_control = YawController(wheel_base, steer_ratio,
                        min_speed, max_lat_accel, max_steer_angle)
        self.filter = LowPassFilter(0.2, 0.1)
        self.last_timestamp = None


    '''
    Params:
    target_v - desired linear velocity
    target_w - desired angular velocity
    current_v - current linear velocity
    dbw_enabled - drive by wire enabled (ignore error in this case)
    '''
    def control(self, target_v, target_w, current_v, dbw_enabled):
        #rospy.loginfo("target_v %s, current_v %s", target_v.x, current_v.x)
        
        error = min(target_v.x, self.max_speed) - current_v.x
        
        # TODO: Change the arg, kwarg list to suit your needs. Return throttle, brake, steer
        if self.last_timestamp is None or not dbw_enabled:
            self.last_timestamp = rospy.get_time()
            # Reset throttle PID, steer filter
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.filter.reset()
            return 0., 0.1, 0.

        # Car is stopped or approaching red light
        if target_v.x < 1e-5:  # shall avoid very slow speed being recognized as stopping
            self.last_timestamp = rospy.get_time()
            # Reset throttle PID, steer filter
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.filter.reset()
            return 0, 5000.0, 0
  

        ### Only one of throttle & brake allowed to be non-Zero
        dt = rospy.get_time() - self.last_timestamp
        # braking
        if error < 0:
            brake = self.brake_pid.step(-1.0*error, dt)
            brake = max(brake, 1.0)
            # Reset throttle PID
            self.throttle_pid.reset()
            throttle = 0.0
        else:
            # PID
            throttle = self.throttle_pid.step(error, dt)
            throttle = min(max(throttle, 0.), self.accel_limit)
            # Reset brake PID
            self.brake_pid.reset()
            brake = 0.0

        # When brake, no steering
        if error < 0:
            self.filter.reset()
            steering = 0.
        else:
            steering = self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x)
            steering = self.filter.filt(steering)
            steering = min(max(steering, -7.), 7.)
        
        self.last_timestamp = rospy.get_time()
        
        #if not brake == 0.0:
        #    rospy.loginfo("Controller: target_v %s, error %s, caculated throttle %s, steering %s, brake %s", target_v.x, error, throttle, steering, brake)
        return throttle, brake, steering
