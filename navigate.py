'''
Created on Nov 29, 2019

@author: Shu You
'''

import carla
import time

def main():
    try:
        client = carla.Client('127.0.0.1', 2000) # use localhost and port 2000
        client.set_timeout(2.0)
        world = client.get_world()
        spectator = world.get_spectator()

        time.sleep(3)

        
        while True:
            my_transform = spectator.get_transform()
            nearest_waypoint = world.get_map().get_waypoint(my_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            print(nearest_waypoint)
    except (KeyboardInterrupt):
        print("clearing")
        print("bye")

if __name__ == '__main__':
    main()