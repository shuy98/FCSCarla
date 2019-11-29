'''
Created on Nov 21, 2019

@author: Shu You

World interaction and main control algorithm implementation.
'''

import carla
from vehicle import MyVehicle
import math
import time

'''
Find the nearest waypoint to the vehicle from the predefined trajectory
'''
def find_min(my_waypoints, vehicle_trans):
    vehicle_x = vehicle_trans.location.x
    vehicle_y = vehicle_trans.location.y

    if (len(my_waypoints) != 0):
        min_waypoint = my_waypoints[0]
        min_waypoint_x = float(my_waypoints[0].split(" ")[0])
        min_waypoint_y = float(my_waypoints[0].split(" ")[1])
        min_val = ((vehicle_x - min_waypoint_x) ** 2 + (vehicle_y - min_waypoint_y) ** 2) ** 0.5
    else:
        return None
    
    for i in my_waypoints:
        waypoint_x = float(i.split(" ")[0])
        waypoint_y = float(i.split(" ")[1])
        
        val = ((vehicle_x - waypoint_x) ** 2 + (vehicle_y - waypoint_y) ** 2) ** 0.5
        if (val < min_val):
            min_val = val
            min_waypoint = i
            
    temp_idx = my_waypoints.index(min_waypoint)
    
    if (temp_idx < len(my_waypoints) - 1):
        return my_waypoints[temp_idx + 1]
    return min_waypoint

def main():
    try:
        client = carla.Client('127.0.0.1', 2000) # use localhost and port 2000
        client.set_timeout(2.0)
        world = client.get_world()
        spectator = world.get_spectator()
        actor_list = []
        
        my_vehicle_obj = MyVehicle(world, spectator) # instantiate a new vehicle
        my_vehicle_obj.drop_vehicle(spectator.get_location()) # drop at my spectator location
        my_vehicle = my_vehicle_obj.get_vehicle()
        actor_list.append(my_vehicle)

        time.sleep(3)

        my_vehicle.apply_control(carla.VehicleControl(throttle=0.4))
        
        # load predefined waypoints
        my_waypoints = []
        with open("./compose.txt", "r") as f:
            for line in f:
                my_waypoints.append(line)
            
        while True:
            vehicle_transform = my_vehicle.get_transform()
            
            '''
            Find the nearest waypoint based on vehicle location
            '''
            next_waypoint = find_min(my_waypoints, vehicle_transform)
            
            next_x = float(next_waypoint.split(" ")[0])
            next_y = float(next_waypoint.split(" ")[1])
            next_yaw = float(next_waypoint.split(" ")[2])
            
            vehicle_pos = vehicle_transform.location
            
            road_yaw = next_yaw / 180 * math.pi
            vehicle_yaw = vehicle_transform.rotation.yaw / 180 * math.pi

            '''
            Heading error calculation
            '''
            psi = -vehicle_yaw + road_yaw
            if (psi > math.pi):
                psi = psi - 2 * math.pi
            elif (psi < -math.pi):
                psi = psi + 2 * math.pi

            wheel_mid_pos_x = vehicle_pos.x + 1.5 * math.cos(vehicle_yaw)
            wheel_mid_pos_y = vehicle_pos.y + 1.5 * math.sin(vehicle_yaw)

            '''
            Cross track error calculation
            '''
            slope = math.tan(road_yaw)
            c = next_y - next_x * slope            
            lateral_error = math.fabs(((slope * wheel_mid_pos_x - wheel_mid_pos_y + c))) / math.sqrt(slope ** 2 + 1)            
            if ((road_yaw > 0 and road_yaw < math.pi / 2) or (road_yaw < 0 and road_yaw > -math.pi / 2)):
                lateral_error = -lateral_error

            '''
            Vehicle speed calculation
            '''
            vehicle_velo_vec = my_vehicle.get_velocity()
            vehicle_velo = (vehicle_velo_vec.x ** 2 + vehicle_velo_vec.y ** 2) ** 0.5
            
            '''
            Controller gain
            '''
            k = -0.2
            
            '''
            Steering input
            '''
            steer_angle = psi + math.atan2(k * lateral_error, vehicle_velo)
            steer_limit = 70.0 / 180 * math.pi
            if (steer_angle > steer_limit):
                steer_val = 1.0
            elif (steer_angle < -steer_limit):
                steer_val = -1.0
            else:
                steer_val = steer_angle / steer_limit
            
            '''
            Execute control command
            '''
            my_vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=steer_val))

    except (KeyboardInterrupt):
        print("clearing")
        for a in actor_list:
            a.destroy()
        print("bye")

if __name__ == '__main__':
    main()
