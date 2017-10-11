import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 40.0

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, 
                    min_speed, max_lat_accel, max_steer_angle,
                    throttle_coeff, steering_coeff):
        # TODO: Implement
        self.throttle_pid = PID(throttle_coeff[0],throttle_coeff[1],throttle_coeff[2])
        self.yaw_control = YawController(wheel_base, steer_ratio,
                        min_speed, max_lat_accel, max_steer_angle)
                        #steering_coeff)
        self.filter = LowPassFilter(0.2,0.1)
        self.last_timestamp = None

    '''
    Params:
    target_v - desired linear velocity
    target_w - desired angular velocity
    current_v - current linear velocity
    dbw_enabled - drive by wire enabled (ignore error in this case)
    '''
    def control(self, target_v, target_w, current_v, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs. Return throttle, brake, steer
        # first time in or dbw not enabled
        if self.last_timestamp is None or not dbw_enabled:
            self.last_timestamp = rospy.get_time()
            return 0., 0., 0.
        
        ### only one of throttle & brake allowed to be non-Zero
        # stopping
        if abs(target_v.x) < 0.1:
            brake = 12.0
            throttle = 0.0
        else:
            error = min(target_v.x, MAX_SPEED*ONE_MPH) - current_v.x
            dt = rospy.get_time() - self.last_timestamp
            # PID
            throttle = self.throttle_pid.step(error, dt)
            throttle = max(min(throttle, 0.3), 1.6)
            brake = 0.0
        
        steering = self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x)
        steering = self.filter.filt(steering)

        self.last_timestamp = rospy.get_time()

        return throttle, brake, steering
