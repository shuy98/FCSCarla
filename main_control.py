'''
Created on Nov 21, 2019

@author: Shu You

World interaction and main control algorithm implementation.
'''

import carla
from vehicle import MyVehicle
import math
import time

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
        
        
        while True:
            vehicle_transform = my_vehicle.get_transform()
            nearest_waypoint = world.get_map().get_waypoint(vehicle_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    #         for w in nearest_waypoint.next(1.0):
    #             print(w.transform)
    #         print()
#             if (nearest_waypoint.get_left_lane() != None):
#                 next_waypoint = nearest_waypoint.get_left_lane()
#             else:
#             next_waypoint = nearest_waypoint.next(2)[0]
            next_waypoint = nearest_waypoint
    #         print("ref position %f" % lateral_pos)
    #         print("my position %f" % vehicle_transform.location.y)
    #         print()
            
            lateral_pos = next_waypoint.transform.location
            vehicle_pos = vehicle_transform.location
            
            road_yaw = next_waypoint.transform.rotation.yaw / 180 * math.pi
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
            c = lateral_pos.y - lateral_pos.x * slope
            
            lateral_error = ((slope * wheel_mid_pos_x - wheel_mid_pos_y + c)) / math.sqrt(slope ** 2 + 1)       
            print(lateral_error)     
            #############



            #lateral_error = ((lateral_pos.x - wheel_mid_pos_x) ** 2 + (lateral_pos.y - wheel_mid_pos_y) ** 2) ** 0.5
#             if (wheel_mid_pos_y < lateral_pos.y):
#                 lateral_error = -lateral_error
            
    #         lateral_error = - lateral_pos + vehicle_transform.location.y # need to revise, should take center of the front wheels
        
            vehicle_velo_vec = my_vehicle.get_velocity()
            vehicle_velo = (vehicle_velo_vec.x ** 2 + vehicle_velo_vec.y ** 2) ** 0.5
    #         print(vehicle_velo)
            
            k = 0.25
            steer_angle = psi + math.atan2(k * lateral_error, vehicle_velo)
            steer_limit = 70.0 / 180 * math.pi
            if (steer_angle > steer_limit):
                steer_val = 1.0
            elif (steer_angle < -steer_limit):
                steer_val = -1.0
            else:
                steer_val = steer_angle / steer_limit
            
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