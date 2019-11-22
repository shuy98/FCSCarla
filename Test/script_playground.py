'''
Created on Nov 20, 2019

@author: shuyo
'''
import carla

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)

print(client.get_available_maps())

world = client.get_world()
myMap = world.get_map()
print(myMap)