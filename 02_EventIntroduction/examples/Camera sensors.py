import sys, os
sys.path.append("D:\Projects\CARLA-FY2022\carla")
sys.path.append("D:\Projects\CARLA-FY2022\carla\dist\carla-0.9.11-py3.7-win-amd64.egg")

import carla
import math
import random
import time
import numpy as np
import cv2


# Connect the client and set up bp library and spawn points
client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

# Add the ego vehicle
vehicle_bp = bp_lib.find('vehicle.toyota.prius')
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

# Move the spectator behind the vehicle to view it
spectator = world.get_spectator()
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation)
spectator.set_transform(transform)

# Add traffic
for i in range(50):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))


# Set traffic in motion
for v in world.get_actors().filter('*vehicle*'):
    v.set_autopilot(True)

# Set initial camera translation
camera_init_trans = carla.Transform(carla.Location(z=2))

# Add one of each type of camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

sem_camera_bp = bp_lib.find('sensor.camera.semantic_segmentation')
sem_camera = world.spawn_actor(sem_camera_bp, camera_init_trans, attach_to=vehicle)


# Define respective callbacks
def rgb_callback(image, data_dict):
    data_dict['rgb_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))


def sem_callback(image, data_dict):
    image.convert(carla.ColorConverter.CityScapesPalette)
    data_dict['sem_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))


# Initialise parameters and data
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
               'sem_image': np.zeros((image_h, image_w, 4))}

# OpenCV named window for display
cv2.namedWindow('All cameras', cv2.WINDOW_AUTOSIZE)

# Tile all data in one array
top_row = np.concatenate((sensor_data['rgb_image'], sensor_data['sem_image']), axis=1)

# Display with imshow
cv2.imshow('All cameras', top_row)
cv2.waitKey(1)

# Set sensors recording
camera.listen(lambda image: rgb_callback(image, sensor_data))
sem_camera.listen(lambda image: sem_callback(image, sensor_data))

# Indefinite while loop
while True:

    # Tile camera images into one array
    top_row = np.concatenate((sensor_data['rgb_image'], sensor_data['sem_image']), axis=1)

    # Dispaly with imshow
    cv2.imshow('All cameras', top_row)

    # Break loop if user presses q
    if cv2.waitKey(1) == ord('q'):
        break

# Stop sensors and destroy OpenCV window
camera.stop()
sem_camera.stop()
cv2.destroyAllWindows()
