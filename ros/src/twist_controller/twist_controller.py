import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import sys

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, 
                    max_speed, min_speed, accel_limit, decel_limit,
                    max_lat_accel, max_steer_angle, brake_deadband):
        # TODO: Implement
        self.max_speed = max_speed
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.max_steer_angle = max_steer_angle
        self.brake_deadband = brake_deadband
        # Use respective PID for brake & throttle, since they give different acceleration
        self.throttle_pid = PID(4.0, 0.5, 0.0)
        self.brake_pid = PID(440.0, 0.0, 0.0)
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
        #if current_v.x < 0.2:
        #    rospy.loginfo("target_v %s, current_v %s", target_v.x, current_v.x)
        #    sys.stdout.flush()
        
        # TODO: Change the arg, kwarg list to suit your needs. Return throttle, brake, steer
        if self.last_timestamp is None or not dbw_enabled:
            self.last_timestamp = rospy.get_time()
            # Reset throttle PID, steer filter
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.filter.reset()
            return 0., 0., 0.

        # Car is stopped or approaching stopped
        if not abs(target_v.x - self.max_speed) < 1e-5 and current_v.x < 10:
            self.last_timestamp = rospy.get_time()
            # Reset throttle PID, steer filter
            self.throttle_pid.reset()
            self.brake_pid.reset()
            self.filter.reset()
            return 0, 10000, 0
 
        
        error = min(target_v.x, self.max_speed) - current_v.x
        dt = rospy.get_time() - self.last_timestamp

        # braking
        if error < 0:
            brake = self.brake_pid.step(-1.0 * error, dt)
            brake = max(1, brake)
            #rospy.loginfo("Controller: current_v_v %s, error %s, brake %s", current_v.x, error,  brake)
            #sys.stdout.flush()
            # Reset throttle PID
            self.throttle_pid.reset()
            throttle = 0.0
        else:
            # PID
            throttle = self.throttle_pid.step(error, dt)
            throttle = min(max(throttle, self.decel_limit), self.accel_limit)
            # Reset brake PID
            self.brake_pid.reset()
            brake = 0.0

        # when brake , no steering
        if error < 0: #not abs(target_v.x - self.max_speed) < 1e-5 : 
            steering = 0.
            self.filter.reset()
        else:
            steering = self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x)
            steering = self.filter.filt(steering)
            steering = min(max(steering, -1.0*self.max_steer_angle), self.max_steer_angle)
        
        self.last_timestamp = rospy.get_time()
        
        #rospy.loginfo("Controller: target_v %s, error %s, brake %s", target_v.x, error,  brake)
        #sys.stdout.flush()
        return throttle, brake, steering
