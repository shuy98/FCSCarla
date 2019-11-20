'''
Created on Nov 20, 2019

@author: shuyo
'''

import carla
import math
import random

def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def main():
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

if __name__ == '__main__':
    main()