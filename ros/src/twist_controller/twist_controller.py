from yaw_controller import *
from pid import *
from lowpass import *
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                                    wheel_base, steer_ratio, max_lat_accel, max_steer_angle, max_velocity_launch):
        # TODO: Implement
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # parameter explain:
        # wheel_base: measured between rotational centers of wheels
        # steer_ratio: ratio between the turn of the steering wheel and the turn of the wheels
        # max_lat_accel: maximum lateral acceleration 
        
        kp = 0.35
        ki = 0.1
        kd = 0.0


        mn = 0.0 #min throttle value
        mx = 0.4 #max throttle value

        if (max_velocity_launch <= 10):
            mx = 0.7
        elif (max_velocity_launch > 10 and max_velocity_launch <= 25):
            mx = 0.6
        elif (max_velocity_launch > 25 and max_velocity_launch <= 35):
            mx = 0.45
        elif (max_velocity_launch > 35 and max_velocity_launch <= 45):
            mx = 0.3
        elif (max_velocity_launch > 45):
            mx = 0.2        
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        # PID controller parameter: 
        # P-controller: gives output which is proportional to current error.
        # I-controller: provides necessary action to eliminate the steady state error
        # D-controller: anticipates future behavior of the error
        # reference: https://www.elprocus.com/the-working-of-a-pid-controller/
        
        #===================low pass filter===================#
        tau = 0.5 # highest frequency: 1/(2*PI*tau)
        ts = 0.02 # 50 Hz
        self.low_pass_filter = LowPassFilter(tau, ts) # in order to filter out all of the high-frequency noise in velocity
        #===================low pass filter===================#
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time() # the time of last controling: used in PID controller
        # TODO: END

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled, current_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled: # wait for traffic light, or operated by human driver, then reset controller
            self.throttle_controller.reset() #class reset: set all members to their initial value 
            return 0.0,0.0,0.0
        
        #===================== get steering and throttle value from controllers ==================#
        filtered_current_velocity = self.low_pass_filter.filt(current_velocity)

        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, filtered_current_velocity)

        velocity_error = linear_velocity - filtered_current_velocity
        # difference between desired velocity and current velocity, used in PID controller
        
        self.last_velocity = filtered_current_velocity
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(velocity_error, sample_time)
        brake = 0.0
        
        #================== according to throttle and desired velocity, tuning brake ===============#
        if linear_velocity == 0 and current_velocity < 0.1:
            throttle = 0.0
            brake = 400 # want the car stop: max torque (N*m), 700 = 0.2413 * 1736 * a => a = 1.67
            
        if throttle < 0.1 and velocity_error < 0.0: # means we want to slow down
            throttle = 0.0
            deceleration = max(self.decel_limit, velocity_error)
            # notice: decel_limit is negative, so here we use max(), instead of min() 
            
            brake = abs(deceleration)*self.vehicle_mass*self.wheel_radius
        #rospy.logwarn("throttle: {0}".format(throttle))
        #rospy.logwarn("brake: {0}".format(brake))
        #rospy.logwarn("steering: {0}".format(steering))
  
        return throttle, brake, steering    
            

        
        # Return throttle, brake, steer
        #return 1., 0., 0.
