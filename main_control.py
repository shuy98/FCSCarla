'''
Created on Nov 21, 2019

@author: Shu You

World interaction and main control algorithm implementation.
'''

import carla
from vehicle import MyVehicle
import math
import time
from time import sleep

def find_next_wp(my_waypoints, vehicle_trans):
    vehicle_x = vehicle_trans.location.x
    vehicle_y = vehicle_trans.location.y
    
    for i in my_waypoints:
        waypoint_x = float(i.split(" ")[0])
        waypoint_y = float(i.split(" ")[1])
        waypoint_yaw = float(i.split(" ")[2]) / 180 * math.pi
        
        is_passed = False
        dir = math.atan2(waypoint_y - vehicle_y, waypoint_x - vehicle_x)
        
        if ((waypoint_yaw > 0 and waypoint_yaw < math.pi / 2) or (waypoint_yaw < 0 and waypoint_yaw > -math.pi / 2)):
            if ((dir > 0 and dir < math.pi / 2) or (dir < 0 and dir > -math.pi / 2)):
                is_passed = False
            else:
                is_passed = True
        else:
            if ((dir > 0 and dir < math.pi / 2) or (dir < 0 and dir > -math.pi / 2)):
                is_passed = True
            else:
                is_passed = False
                
        if (not is_passed):
            return i
        else:
            continue

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
#                 if (count < 72):
#                     count+=1
#                 else:
                my_waypoints.append(line)
            
#         prev_point = my_waypoints[0]
        while True:
            vehicle_transform = my_vehicle.get_transform()
            nearest_waypoint = world.get_map().get_waypoint(vehicle_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            
            print("nearest waypoint", nearest_waypoint.transform.rotation.yaw)
    #         for w in nearest_waypoint.next(1.0):
    #             print(w.transform)
    #         print()
#             if (nearest_waypoint.get_left_lane() != None):
#                 next_waypoint = nearest_waypoint.get_left_lane()
#             else:
#             next_waypoint = nearest_waypoint.next(2)[0]

            next_waypoint = find_min(my_waypoints, vehicle_transform)
            
            
#             next_waypoint = find_next_wp(my_waypoints, vehicle_transform)
            print("min waypoint = ", next_waypoint)
            print("vehicle pos = ", vehicle_transform.location)
            
            
            
            next_x = float(next_waypoint.split(" ")[0])
            next_y = float(next_waypoint.split(" ")[1])
            next_yaw = float(next_waypoint.split(" ")[2])
            
            print("next_x", next_x)
            print("next_y", next_y)
            print("next_yaw", next_yaw)
    #         print("ref position %f" % lateral_pos)
    #         print("my position %f" % vehicle_transform.location.y)
    #         print()
            
            map_val = ((vehicle_transform.location.x - nearest_waypoint.transform.location.x) ** 2 + (vehicle_transform.location.y - nearest_waypoint.transform.location.y) ** 2) ** 0.5
            def_val = ((vehicle_transform.location.x - next_x) ** 2 + (vehicle_transform.location.y - next_y) ** 2) ** 0.5
            map_val = -1
            
            if (map_val > def_val):
                next_x = nearest_waypoint.transform.location.x
                next_y = nearest_waypoint.transform.location.y
                next_yaw = nearest_waypoint.transform.rotation.yaw
            
            
            vehicle_pos = vehicle_transform.location
            
            road_yaw = next_yaw / 180 * math.pi
            vehicle_yaw = vehicle_transform.rotation.yaw / 180 * math.pi
    #         print(vehicle_yaw)
            psi = -vehicle_yaw + road_yaw
            if (psi > math.pi):
                psi = psi - 2 * math.pi
            elif (psi < -math.pi):
                psi = psi + 2 * math.pi

            wheel_mid_pos_x = vehicle_pos.x + 1.5 * math.cos(vehicle_yaw)
            wheel_mid_pos_y = vehicle_pos.y + 1.5 * math.sin(vehicle_yaw)
#             print("longitudinal ego pos", vehicle_pos.x)
#             print("lateral ego pos", vehicle_pos.y)
#             print("longitudinal wheel pos", vehicle_pos.x + 1.44 * math.cos(vehicle_yaw))
#             print("lateral wheel pos", vehicle_pos.y + 1.44 * math.sin(vehicle_yaw))
#             
            ############# error calculation
            slope = math.tan(road_yaw)
            c = next_y - next_x * slope
            
            lateral_error = math.fabs(((slope * wheel_mid_pos_x - wheel_mid_pos_y + c))) / math.sqrt(slope ** 2 + 1)
            
            if ((road_yaw > 0 and road_yaw < math.pi / 2) or (road_yaw < 0 and road_yaw > -math.pi / 2)):
                lateral_error = -lateral_error
             
            print(lateral_error)     
            #############



            #lateral_error = ((lateral_pos.x - wheel_mid_pos_x) ** 2 + (lateral_pos.y - wheel_mid_pos_y) ** 2) ** 0.5
#             if (wheel_mid_pos_y < lateral_pos.y):
#                 lateral_error = -lateral_error
            
    #         lateral_error = - lateral_pos + vehicle_transform.location.y # need to revise, should take center of the front wheels
        
            vehicle_velo_vec = my_vehicle.get_velocity()
            vehicle_velo = (vehicle_velo_vec.x ** 2 + vehicle_velo_vec.y ** 2) ** 0.5
    #         print(vehicle_velo)
            
            k = -0.2
            steer_angle = psi + math.atan2(k * lateral_error, vehicle_velo)
            steer_limit = 70.0 / 180 * math.pi
            if (steer_angle > steer_limit):
                steer_val = 1.0
            elif (steer_angle < -steer_limit):
                steer_val = -1.0
            else:
                steer_val = steer_angle / steer_limit
            
            print("steer_val= ", steer_val) 
            my_vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=steer_val))
            
    #         left_wp = nearest_waypoint.get_left_lane()
    #         right_wp = nearest_waypoint.get_right_lane()
    #         print(left_wp, right_wp)
    except (KeyboardInterrupt):
        print("clearing")
        for a in actor_list:
            a.destroy()
        print("bye")

if __name__ == '__main__':
    main()