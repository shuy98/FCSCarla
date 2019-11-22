'''
Created on Nov 21, 2019

@author: Shu You

World interaction and main control algorithm implementation.
'''

import carla
from vehicle import MyVehicle

def main():
    client = carla.Client('127.0.0.1', 2000) # use localhost and port 2000
    client.set_timeout(2.0)
    world = client.get_world()
    spectator = world.get_spectator()
    actor_list = []
    
    my_vehicle_obj = MyVehicle(world, spectator) # instantiate a new vehicle
    my_vehicle = my_vehicle_obj.get_vehicle()
    actor_list.append(my_vehicle)
    my_vehicle_obj.drop_vehicle(spectator.get_location()) # drop at my spectator location

if __name__ == '__main__':
    main()