from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_control = YawController(
            kwargs['wheel_base'],
            kwargs['steer_ratio'],
            kwargs['min_speed'],
            kwargs['max_lat_accel'],
            kwargs['max_steer_angle']
        )

        self.lpf = LowPassFilter(kwargs['steering_tau'], 1.0 / kwargs['sample_rate'])

        pid_gains = kwargs['throttle_gains']
        self.pid = PID(pid_gains[0], pid_gains[1], pid_gains[2], kwargs['max_speed'])

        total_mass = GAS_DENSITY * kwargs['fuel_capacity'] + kwargs['vehicle_mass']
        self.max_brake_torque = total_mass * kwargs['decel_limit'] * kwargs['wheel_radius']
        self.min_brake = -1.0 * kwargs['brake_deadband']

    def reset(self, *args, **kwargs):
        self.pid.reset()

    def control(self, target, current):
        u = self.pid.step(target.linear.x, current.linear.x, rospy.get_time())

        if u > 0:
            # Accelerate
            throttle = max(0.0, min(1.0, u))
            brake = 0.0
        else:
            # Decelerate
            throttle = 0.0
            brake = self.max_brake_torque * min(self.min_brake, u / self.pid.max_abs_u)

        steering = self.yaw_control.get_steering(target.linear.x, target.angular.z, current.linear.x)
        steering = self.lpf.filt(steering) # Low pass filter step

        return throttle, brake, steering