#imports
#import pandas as pd
import math
pi = math.pi
import numpy as np


#OUTPUTS
time = 0
distance = 0
speed = 0

#CONST
tire_radius = .2667
#motor_torque = 231.00
driver_ratio = 3.53
acc_new = 9.75
##max torque
cf = .71
A = 1.00
crr = .02
mass_with_driver = 300

#functions
def get_motor_speed(current_speed):
    return current_speed/(2*pi*tire_radius)*2*pi*driver_ratio

def get_power(current_motor_torque, current_motor_speed):
    return current_motor_torque*current_motor_speed/1000

def get_drag(cf, A, current_speed):
    return cf*A*(current_speed**2)

def get_rolling_resistance(mass_with_driver, crr):
    return mass_with_driver*9.81*crr

def get_acceleration(current_get_motor_torque, driver_ratio, current_get_rolling_resistance, current_get_drag, tire_radius, mass_with_driver, acc_new):
    a = (current_get_motor_torque*driver_ratio-current_get_rolling_resistance-current_get_drag)/tire_radius/mass_with_driver
    return min(a, acc_new)

def get_speed(initial_acceleration, time_delta, initial_speed):
    return initial_acceleration*time_delta+initial_speed
# time_start = 0
# time_end = 6.5
# time_step = .025
# time_current = 0
# speed_timestep = 23
# speed_last = 0

# # for i in np.linspace(time_start, time_end, num = 261):
# #     time_current = np.round(i, 3) 
# #     print(time_current)

# motor_speed = 4
my_list_motor_speed = [-1.00, .01, 5.31, 16.54, 27.77, 39.00, 50.23, 61.47, 72.70, 83.96, 95.16,
         106.40,117.63,128.86,139.39,150.62,161.85,173.09,184.32,195.55,206.78,218.02
         ,229.25,240.48,251.68,262.94,284.92,295.94,307.17,318.40,328.83,336.23,346.60
         ,357.72,368.95,380.18,391.41,402.64,413.18,424.50,435.64,446.87,458.10,469.24
         ,480.57,491.59,502.81,513.91,525.15,536.38,547.47,557.79,568.99]

# my_list_motor_speed = [-1.000000000000000000,0.007347072600764310,5.305317759933680000,16.537534475530600000,
# 27.769751187986000000,39.001967910913300000,50.234184623368600000,61.466401335824000000,72.698618048279300000,
# 83.962744472331900000,95.163051483662000000,106.395268164701000000,117.627484908573000000,128.859701652444000000,
# 139.389904804008000000,150.622121547880000000,161.854338187031000000,173.086554930902000000,184.318771674774000000,
# 195.550988418645000000,206.783205057797000000,218.015421801668000000,229.247638545539000000,240.479855289410000000,
# 251.680162248381000000,262.944288672433000000,273.591494054475000000,284.917312666505000000,295.938925311740000000,
# 307.171141950892000000,318.403358694763000000,328.829236151646000000,336.228925057235000000,346.600902732559000000,
# 357.716117245953000000,368.948333885104000000,380.180550628975000000,391.412767372847000000,402.644984116718000000,
# 413.175187268282000000,424.503133052697000000,435.639620651305000000,446.871837395176000000,458.104054139048000000,
# 469.240541737655000000,480.568487522070000000,491.590100167306000000,502.809553039104000000,513.914130852811000000,
# 525.146347596682000000,536.378564340554000000,547.470378386907000000,557.789977439836000000,568.990284503526000000]
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

#for key, value in my_dict_motor_curve.items():
#    print (key, value)

#function
def get_motor_torque(motor_speed):
    # print("motor_speed ", motor_speed)
    closest_key = (min(my_list_motor_speed, key=lambda x:abs(x-motor_speed)))
    # print(motor_speed)
    # print(closest_key)
    # print(my_dict_motor_curve[closest_key])
    return my_dict_motor_curve.get(closest_key)

#print ("HERE IS IT!", get_motor_torque(4))

#get to the distance 75m and output the time
#x_f = x_i + v_i*t + 1/2*a_i*t^2 

#initial_velocity = speed

#to do: go through each method, check the right type values being passed in are the values correct. WHen calling, are the values I pass in correct, order

def get_next_distance(initial_distance, initial_velocity, delta_time, initial_acceleration):
    #delta_time = .025
    #initial_distance = 0
    #current_distance = 0
    
    current_distance = initial_distance + (initial_velocity * delta_time) + (.5 * initial_acceleration * (delta_time**2))
    # print(current_distance)
    return current_distance

#test
#print ("HERE", get_next_distance(0.0737, 1.1782, .025, 9.4137))
#print("DISTANCE 1:", get_next_distance(0.0472, .9429, .025, 9.4137))
#print("DISTANCE 2:", get_next_distance(0.07, 1.18, .025, 9.41))
# print(get_next_distance(0.05, .94, .025, 9.41))
# print(get_next_distance(0, 0, .025, 9.46))

# time_initial = 0
# initial_distance = 0 
# initial_velocity = 0 #?
# delta_time =.025 
# initial_acceleration = 9.46 #?


def get_time_given_distance(initial_distance, initial_velocity, delta_time, initial_acceleration, final_distance):
    init_distance = initial_distance
    init_velocity = initial_velocity
    init_acceleration = initial_acceleration
    current_time = 0
    
     
    # #old
    # current_distance = initial_distance
    # current_velocity = initial_velocity
    # current_acceleration = initial_acceleration
    # current_time = 0
     
    while init_distance<final_distance:
        # print("time", current_time)
        # print("distance", current_distance)
        # print("velocity", current_velocity)
        # print("acceleration", current_acceleration)
        # print("drag", get_drag(cf, A, current_velocity))
        # print("motor torque", get_motor_torque(current_velocity))
        # print("___________________")
        
        #old: current_distance = get_next_distance(current_distance, current_velocity, delta_time, current_acceleration) 
        new_distance = get_next_distance(init_distance, init_velocity, delta_time, init_acceleration) 
        new_velocity = get_speed(init_acceleration, delta_time, init_velocity)
        #old: new_velocity = get_speed(current_acceleration, (delta_time), current_velocity)
        current_time += delta_time
        new_acceleration = get_acceleration(get_motor_torque(get_motor_speed(new_velocity)), driver_ratio , get_rolling_resistance(mass_with_driver, crr), get_drag(cf, A, new_velocity), tire_radius, mass_with_driver, acc_new)
        #old: current_acceleration = get_acceleration(get_motor_torque(current_velocity), driver_ratio , get_rolling_resistance(mass_with_driver, crr), get_drag(cf, A, current_velocity), tire_radius, mass_with_driver, acc_new)
        #old: current_velocity = new_velocity
        init_distance = new_distance
        init_velocity = new_velocity
        init_acceleration = new_acceleration
    print (current_time)
    return(current_time)
        
    
get_time_given_distance(0, 0, .025, 9.45594300712411, 75)
    
    
    
    


