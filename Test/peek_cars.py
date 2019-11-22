'''
Created on Nov 20, 2019

@author: Shu You

For testing purpose only. Test the simulator.
'''

import carla
import random

def main():
    try:
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        spectator = world.get_spectator()
        actor_list = [];
    
        vehicle_blueprints = world.get_blueprint_library().find("vehicle.ford.mustang")
        if vehicle_blueprints.has_attribute('color'):
                color = random.choice(vehicle_blueprints.get_attribute('color').recommended_values)
                vehicle_blueprints.set_attribute('color', color)
    
        location = spectator.get_location()
        transform = carla.Transform(location, carla.Rotation())
    
    
        vehicle = world.spawn_actor(vehicle_blueprints, transform) # place the vehicle
    
        actor_list.append(vehicle) # add to actor list
        print('created %s' % vehicle.type_id)
        
        
        myMap = world.get_map()

        # attach the sensor to the vehicle
                
        # Retrieve the closest waypoint using sensor.
        # waypoint = 
        
        # control command 
        # vehicle.apply_control(carla.VehicleControl(throttle=0.25, steer=steer_angle, manual_gear_shift=False, gear=3))
            
            
    
    except:
        for a in actor_list:
            a.destroy()
            print("cleared up")

if __name__ == '__main__':
    main()