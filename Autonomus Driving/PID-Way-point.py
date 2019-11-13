# !/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time

import controller


def main():
    actor_list = []
    try:

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()
        Weather = carla.WeatherParameters(cloudyness=10.0, precipitation=30.0, sun_altitude_angle=70.0)  # set weather

        world.set_weather(Weather)
        #  downloading a 300 random cars in the town in a randoum waypoints
        Cars = []
        Car_Count = 0  # 300 is no. of the cars
        for i in range(0, 300):  # generate 300 random cars
            bp_Temp = random.choice(blueprint_library.filter('vehicle'))
            transform_Temp = random.choice(world.get_map().get_spawn_points())
            vehicle_Temp = world.try_spawn_actor(bp_Temp, transform_Temp)
            # check if there isnt a car in this place to can download car in this place or not
            if vehicle_Temp != None:
                Cars.append(vehicle_Temp)
                actor_list.append(vehicle_Temp)
                vehicle_Temp.set_autopilot(True)
                Car_Count = Car_Count + 1
                print('%s' % Car_Count)

        # download my car in a certain waypoint
        bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
        transform = carla.Transform(carla.Location(x=-355, y=26.5, z=1.2),
                                    carla.Rotation(0, -0.37, 0))  # location of our car
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        vehicle.set_autopilot(False)

        PID = controller.VehiclePIDController(vehicle, args_lateral={'K_P': 0.8, 'K_D': 0.001, 'K_I': 0.1},
                                              args_longitudinal={'K_P': 0.8, 'K_D': 0.001,
                                                                 'K_I': 0.1})  # moving my car in pid controller
        map = world.get_map()

        Flag_Turn_1 = 0
        Flag_Turn_2 = 0

        while True:  # calculate the waypoint of the random cars and put them in a list
            Car_Locations = []
            for i in range(0, Car_Count):
                Car_Locations.append(Cars[i].get_location())

            waypoint = map.get_waypoint(vehicle.get_location(), project_to_road=True,
                                        lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
            waypoint_list = waypoint.next(0.5)  # get the next waypoint of my current waypoint in .5 meter

            Waypoints_Locations = []
            for i in range(0, 200):  # 50
                waypoint_list = waypoint_list[0].next(
                    0.1)  # get the next of my current waypoint and return it in a list
                Temp = waypoint_list[0].transform.location
                Waypoints_Locations.append(Temp)

            waypoint_list = waypoint_list[0].next(35)  # 50
            Temp_1 = waypoint_list[0].transform.location
            waypoint_list = waypoint_list[0].next(0.1)
            Temp_2 = waypoint_list[0].transform.location

            Distance_Flag = 0  # check that the distance between may car and the other < 0.1.So,lane overtaking occur

            for i in range(0, Car_Count):
                for j in range(0, 200):  # 50
                    Distance = math.sqrt(((Car_Locations[i].x) - (Waypoints_Locations[j].x)) ** 2 + (
                                (Car_Locations[i].y) - (Waypoints_Locations[j].y)) ** 2 + (
                                                     (Car_Locations[i].z) - (Waypoints_Locations[j].z)) ** 2)
                    if Distance < 0.1:
                        Distance_Flag = 1
                        break
                if Distance_Flag == 1:
                    break

            print('%s' % Distance_Flag)

            # print('%s' % abs(Waypoints_Locations[48].x - Waypoints_Locations[49].x))
            # print('%s' % abs(Waypoints_Locations[48].y - Waypoints_Locations[49].y))

            # check if there is a turn or not
            if Flag_Turn_1:
                Flag_Turn_2 = Flag_Turn_2 + 1

            if abs(Temp_1.x - Temp_2.x) < 0.09 and abs(Temp_1.y - Temp_2.y) < 0.09:
                Flag_Turn_1 = 1
                Flag_Turn_2 = Flag_Turn_2 + 1

            if Flag_Turn_2 == 75:
                Flag_Turn_1 = 0
                Flag_Turn_2 = 0

            # print('%s' % Flag_Turn_1)
            # print('%s' % Flag_Turn_2)

            # check if the road is impty or not
            if Distance_Flag == 0:
                '''
                for i in range (0,100) :
                    waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
                    waypoint_list = waypoint.next(4.0)
                    vehicle.apply_control(PID.run_step(100,waypoint_list[0]))
                    time.sleep(0.001)
                '''
                # if there is a turn
                if Flag_Turn_1:
                    waypoint = map.get_waypoint(vehicle.get_location(), project_to_road=True,
                                                lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
                    waypoint_list = waypoint.next(3.0)
                    vehicle.apply_control(PID.run_step(60, waypoint_list[0]))
                    time.sleep(0.1)
                else:
                    waypoint = map.get_waypoint(vehicle.get_location(), project_to_road=True,
                                                lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
                    waypoint_list = waypoint.next(5.0)
                    vehicle.apply_control(PID.run_step(100, waypoint_list[0]))
                    time.sleep(0.1)
            else:  # stop the car
                vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=1))

        # trial to do the lane overtacking

        '''
            map = world.get_map()
        waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        waypoint_list = waypoint.next(2.0)
        PID = controller.VehiclePIDController(vehicle, args_lateral={'K_P': 1.0, 'K_D': 0.001, 'K_I': 1.0}, args_longitudinal={'K_P': 1.0, 'K_D': 0.001, 'K_I': 1.0})
        vehicle.apply_control(PID.run_step(10,waypoint_list[0]))
        time.sleep(0.1)
    
        for i in range (0,100) :
            waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
            waypoint_list = waypoint.next(2.0)
            vehicle.apply_control(PID.run_step(40,waypoint_list[0]))
            time.sleep(0.1)
    
        waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        right_lane=waypoint.get_right_lane()
        waypoint_list = right_lane.next(4.0)
    
        for i in range (0,100) :
            vehicle.apply_control(PID.run_step(40,waypoint_list[0]))
            time.sleep(0.04)
            waypoint_list = waypoint_list[0].next(0.5)
    
    
    
        for i in range (0,100) :
            waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
            waypoint_list = waypoint.next(2.0)
            vehicle.apply_control(PID.run_step(40,waypoint_list[0]))
            time.sleep(0.1)
    
    
    
    
        waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
        left_lane=waypoint.get_left_lane()
        waypoint_list = left_lane.next(4.0)
    
        for i in range (0,100) :
            vehicle.apply_control(PID.run_step(40,waypoint_list[0]))
            time.sleep(0.04)
            waypoint_list = waypoint_list[0].next(0.5)
    
    
    
        for i in range (0,100) :
            waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
            waypoint_list = waypoint.next(2.0)
            vehicle.apply_control(PID.run_step(40,waypoint_list[0]))
            time.sleep(0.1)
        '''

        time.sleep(30)

    finally:

        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')


if __name__ == '__main__':
    main()




