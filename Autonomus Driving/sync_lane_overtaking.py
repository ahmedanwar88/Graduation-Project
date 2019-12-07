#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math

import random
import controller
try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def main():
    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (800, 600),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    try:
	'''
	client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
	'''
        blueprint_library = world.get_blueprint_library()
	Weather = carla.WeatherParameters(
  	  cloudyness=10.0,
  	  precipitation=30.0,
   	 sun_altitude_angle=70.0)
	world.set_weather(Weather)
	Cars = []
	Car_Count = 0 # 300
	for i in range (0,1) : # generate 300 random cars
		bp_Temp = random.choice(blueprint_library.filter('vehicle'))
		#transform_Temp = random.choice(world.get_map().get_spawn_points())
		transform_Temp = carla.Transform(carla.Location(x=-300, y=35, z=1.2),carla.Rotation(0,-0.37,0))
		vehicle_Temp = world.try_spawn_actor(bp_Temp, transform_Temp)
		if  vehicle_Temp != None : 
			Cars.append(vehicle_Temp)
			actor_list.append(vehicle_Temp)
			vehicle_Temp.set_autopilot(False)
		        vehicle_Temp.apply_control(carla.VehicleControl(throttle=0))
			Car_Count=Car_Count+1
			print('%s' % Car_Count)
	

	'''
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.*')),
            start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)
	'''

	
	blueprint_library = world.get_blueprint_library()
	bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
	transform  = carla.Transform(carla.Location(x=-355, y=26.5, z=1.2),carla.Rotation(0,-0.37,0)) #location of our car
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
	vehicle.set_autopilot(False)
	
	PID = controller.VehiclePIDController(vehicle, args_lateral={'K_P': 0.8, 'K_D': 0.001, 'K_I': 0.1}, args_longitudinal={'K_P': 0.8, 'K_D': 0.001, 'K_I': 0.1})
	map = world.get_map()
	Flag_Turn_1 = 0 #
	Flag_Turn_2 = 0

	while True :
		Car_Locations = []
		for i in range (0,Car_Count) :
			Car_Locations.append( Cars[i].get_location() )
	

		waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
		waypoint_list = waypoint.next(0.5)	

		Waypoints_Locations = []
		for i in range (0,200) :   #50
			waypoint_list = waypoint_list[0].next(0.1) #get the next of my current waypoint and return it in a list 
			Temp = waypoint_list[0].transform.location
			Waypoints_Locations.append( Temp )

		waypoint_list = waypoint_list[0].next(35)    #50
		Temp_1 = waypoint_list[0].transform.location
		waypoint_list = waypoint_list[0].next(0.1)
		Temp_2 = waypoint_list[0].transform.location
		

		Distance_Flag = 0 #check that the distance between may car and the other < 0.1.So,lane overtaking occur 

		for i in range (0,Car_Count) :
			for j in range (0,200) :  #50
				Distance = math.sqrt(((Car_Locations[i].x)-(Waypoints_Locations[j].x))**2 + ((Car_Locations[i].y)-(Waypoints_Locations[j].y))**2 + ((Car_Locations[i].z)-(Waypoints_Locations[j].z))**2)
				if Distance < 0.1 :
					Distance_Flag = 1
					break
			if Distance_Flag == 1 :
				break

		print('%s' % Distance_Flag)

		#print('%s' % abs(Waypoints_Locations[48].x - Waypoints_Locations[49].x))
		#print('%s' % abs(Waypoints_Locations[48].y - Waypoints_Locations[49].y))
		'''
		if Flag_Turn_1 :
			Flag_Turn_2 = Flag_Turn_2 + 1

		if abs(Temp_1.x - Temp_2.x) < 0.09 and abs(Temp_1.y - Temp_2.y) < 0.09 :
			Flag_Turn_1 = 1
			Flag_Turn_2 = Flag_Turn_2 + 1

		if Flag_Turn_2 == 75 :
			Flag_Turn_1 = 0
			Flag_Turn_2 = 0

		#print('%s' % Flag_Turn_1)
		#print('%s' % Flag_Turn_2)
		'''

		
		'''
		if Distance_Flag == 0 :
			
			for i in range (0,100) :
				waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
				waypoint_list = waypoint.next(4.0)
				vehicle.apply_control(PID.run_step(100,waypoint_list[0]))
				
			

			if Flag_Turn_1 :
				waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
				waypoint_list = waypoint.next(3.0)
				vehicle.apply_control(PID.run_step(60,waypoint_list[0]))
				#time.sleep(0.1)
				#sync.tick(timeout=0.1)
				#sync_mode.tick(timeout=0.1)
			else :
				waypoint = map.get_waypoint(vehicle.get_location(),project_to_road=True,lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
				waypoint_list = waypoint.next(5.0)
				vehicle.apply_control(PID.run_step(100,waypoint_list[0]))
				#time.sleep(0.1)
				#sync.tick(timeout=0.1)
				#sync_mode.tick(timeout=0.1)
				'''
		#else :
	vehicle.apply_control(carla.VehicleControl(throttle=0,steer=0,brake=1))


        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_rgb)

        camera_semseg = world.spawn_actor(
            blueprint_library.find('sensor.camera.semantic_segmentation'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=vehicle)
        actor_list.append(camera_semseg)

        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_rgb, camera_semseg, fps=30) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                # Advance the simulation and wait for the data.
                snapshot, image_rgb, image_semseg = sync_mode.tick(timeout=20.0)

                # Choose the next waypoint and update the car location.
                waypoint = random.choice(waypoint.next(1.5))
                vehicle.set_transform(waypoint.transform)

                image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                draw_image(display, image_rgb)
                draw_image(display, image_semseg, blend=True)
                display.blit(
                    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    (8, 28))
                pygame.display.flip()

    finally:

        print('destroying actors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
