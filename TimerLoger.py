import math
pi = math.pi
#import numpy as np

class TimeLogger:
    #CONST
    def __init__(self, initial_distance, initial_velocity, delta_time, initial_acceleration, final_distance):
        self.initial_distance = initial_distance
        self.initial_velocity = initial_velocity
        self.delta_time = delta_time
        self.initial_acceleration = initial_acceleration
        self.final_distance = final_distance 
        self.tire_radius = .2667
        self.driver_ratio = 3.53
        self.acc_new = 9.75
        self.cf = .71
        self.A = 1.00
        self.crr = .02
        self.mass_with_driver = 300
        
    #functions
    def get_motor_speed(self, current_speed):
        return current_speed/(2*pi*self.tire_radius)*2*pi*self.driver_ratio

    def get_power(self, current_motor_torque, current_motor_speed):
        return current_motor_torque*current_motor_speed/1000

    def get_drag(self, cf, A, current_speed):
        return cf*A*(current_speed**2)

    def get_rolling_resistance(self, mass_with_driver, crr):
        return mass_with_driver*9.81*crr

    def get_acceleration(self, current_get_motor_torque, driver_ratio, current_get_rolling_resistance, current_get_drag, tire_radius, mass_with_driver, acc_new):
        a = (current_get_motor_torque*driver_ratio-current_get_rolling_resistance-current_get_drag)/tire_radius/mass_with_driver
        return min(a, acc_new)

    def get_speed(self, initial_acceleration, time_delta, initial_speed):
        return initial_acceleration*time_delta+initial_speed
    
    my_list_motor_speed = [-1.00, .01, 5.31, 16.54, 27.77, 39.00, 50.23, 61.47, 72.70, 83.96, 95.16,
            106.40,117.63,128.86,139.39,150.62,161.85,173.09,184.32,195.55,206.78,218.02
            ,229.25,240.48,251.68,262.94,284.92,295.94,307.17,318.40,328.83,336.23,346.60
            ,357.72,368.95,380.18,391.41,402.64,413.18,424.50,435.64,446.87,458.10,469.24
            ,480.57,491.59,502.81,513.91,525.15,536.38,547.47,557.79,568.99]

    my_dict_motor_curve = {-1.00:231.00, 0.01:230.23, 5.31:230.22, 16.54:230.22, 27.77:230.22, 
            39.00:230.22, 50.23:230.22 , 61.47:230.22, 72.70:230.22, 83.96:230.33,
            95.16:230.22 , 106.40:230.22 , 117.63:230.22 , 128.86:213.46 , 139.39:198.97,
            150.62:185.49, 161.85:173.05, 173.09:161.98, 184.32:152.13, 195.55:143.41,
            206.78:135.65, 218.02:128.67, 229.25:122.37, 240.48:116.66, 251.68:111.45,
            262.94:106.69, 273.59:102.51, 284.92:98.45, 295.94:94.78, 307.17:91.31,
            318.40:88.11, 328.83:85.34, 336.23:83.46, 346.60:80.97, 357.72:78.47,
            368.95:76.11, 380.18:73.86, 391.41:71.74, 402.64:69.76, 413.18:67.98,
            424.50:66.18, 435.64:64.49, 446.87:62.89, 458.10:61.38, 469.24:59.98,
            480.57:58.64, 491.59:57.41, 502.81:56.22, 513.91:55.12, 525.15:54.04,
            536.38:53.00, 547.47:52.02, 557.79:51.17, 568.99:50.24}

    #function
    def get_motor_torque(self, motor_speed):
        closest_key = (min(self.my_list_motor_speed, key=lambda x:abs(x-motor_speed)))
        return self.my_dict_motor_curve.get(closest_key)

    def get_next_distance(self, initial_distance, initial_velocity, delta_time, initial_acceleration):
        delta_time = .025
        current_distance = initial_distance + (initial_velocity * delta_time) + (.5 * initial_acceleration * (delta_time**2))
        return current_distance
    
    
    def get_time_given_distance(self, initial_distance, initial_velocity, delta_time, initial_acceleration, final_distance):
        init_distance = initial_distance
        init_velocity = initial_velocity
        init_acceleration = initial_acceleration
        current_time = 0
     
        while init_distance<final_distance:
            new_distance = self.get_next_distance(init_distance, init_velocity, delta_time, init_acceleration) 
            new_velocity = self.get_speed(init_acceleration, delta_time, init_velocity)
            current_time += delta_time
            new_acceleration = self.get_acceleration(self.get_motor_torque(self.get_motor_speed(new_velocity)), self.driver_ratio , self.get_rolling_resistance(self.mass_with_driver, self.crr), self.get_drag(self. cf, self.A, new_velocity), self.tire_radius, self.mass_with_driver, self.acc_new)
            init_distance = new_distance
            init_velocity = new_velocity
            init_acceleration = new_acceleration
        print (current_time)
        return(current_time)
            
    
newTimerLoger = TimeLogger(0, 0, .025, 9.45594300712411, 75)   
newTimerLoger.get_time_given_distance(0, 0, .025, 9.45594300712411, 75)