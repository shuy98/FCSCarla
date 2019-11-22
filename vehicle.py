'''
Created on Nov 21, 2019

@author: Shu You
'''

import carla, random

class MyVehicle(object):
    '''
    Create a new vehicle on the map
    '''
    def __init__(self, world, spectator):
        '''
        Initialize everything.
        world: pass in the world
        spectator: pass in the current spectator
        '''
        self.world = world
        self.spectator = spectator
        self.vehicle_blueprints = self.world.get_blueprint_library().find("vehicle.ford.mustang")
        # pick a random color for the car :)
        if self.vehicle_blueprints.has_attribute('color'):
                color = random.choice(self.vehicle_blueprints.get_attribute('color').recommended_values)
                self.vehicle_blueprints.set_attribute('color', color)
    
    def drop_vehicle(self, location):
        '''
        Drop the vehicle at the specified location
        '''
        transform = carla.Transform(location, carla.Rotation())
        self.vehicle = self.world.spawn_actor(self.vehicle_blueprints, transform) # place the vehicle
        print('created %s' % self.vehicle.type_id)

    def get_vehicle(self):
        '''
        Get the current vehicle
        '''
        return self.vehicle
    
    def attach_sensor(self):
        '''
        TODO: attach the sensor to the vehicle
        '''
        pass
        
        
        
        
        
        
        
        
        