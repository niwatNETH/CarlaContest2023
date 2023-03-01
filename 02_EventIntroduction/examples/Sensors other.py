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
for i in range(200):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Set traffic in motion
for v in world.get_actors().filter('*vehicle*'):
    v.set_autopilot(True)

# Add camera sensor
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Add navigation sensor
gnss_bp = bp_lib.find('sensor.other.gnss')
gnss_sensor = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=vehicle)

# Add inertial measurement sensor
imu_bp = bp_lib.find('sensor.other.imu')
imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

# Add collision sensor
collision_bp = bp_lib.find('sensor.other.collision')
collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)

# Add lane invasion sensor
lane_inv_bp = bp_lib.find('sensor.other.lane_invasion')
lane_inv_sensor = world.spawn_actor(lane_inv_bp, carla.Transform(), attach_to=vehicle)

# Add obstacle detector
obstacle_bp = bp_lib.find('sensor.other.obstacle')
obstacle_bp.set_attribute('hit_radius','0.5')
obstacle_bp.set_attribute('distance','50')
obstacle_sensor = world.spawn_actor(obstacle_bp, carla.Transform(), attach_to=vehicle)


# All sensor callbacks
def rgb_callback(image, data_dict):
    data_dict['rgb_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))


def gnss_callback(data, data_dict):
    data_dict['gnss'] = [data.latitude, data.longitude]


def imu_callback(data, data_dict):
    data_dict['imu'] = {
        'gyro': data.gyroscope,
        'accel': data.accelerometer,
        'compass': data.compass
    }


def lane_inv_callback(event, data_dict):
    data_dict['lane_invasion'] == True


def collision_callback(event, data_dict):
    data_dict['collision'] = True


def obstacle_callback(event, data_dict, camera, k_mat):
    if 'static' not in event.other_actor.type_id:
        data_dict['obstacle'].append({'transform': event.other_actor.type_id, 'frame': event.frame})

    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    image_point = get_image_point(event.other_actor.get_transform().location, k_mat, world_2_camera)
    if 0 < image_point[0] < image_w and 0 < image_point[1] < image_h:
        cv2.circle(data_dict['rgb_image'], tuple(image_point), 10, (0, 0, 255), 3)


# Auxilliary geometry functions for transforming to screen coordinates
def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(loc, K, w2c):
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth componebonent also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return tuple(map(int, point_img[0:2]))


world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

# Get the attributes from the camera
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()

# Calculate the camera projection matrix to project from 3D -> 2D
K = build_projection_matrix(image_w, image_h, fov)

# Initialise data
collision_counter = 20
lane_invasion_counter = 20
sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
               'collision': False,
               'lane_invasion': False,
               'gnss': [0, 0],
               'obstacle': [],
               'imu': {
                   'gyro': carla.Vector3D(),
                   'accel': carla.Vector3D(),
                   'compass': 0
               }}

# OpenCV window with initial data
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera', sensor_data['rgb_image'])
cv2.waitKey(1)

# Start sensors recording data
camera.listen(lambda image: rgb_callback(image, sensor_data))
collision_sensor.listen(lambda event: collision_callback(event, sensor_data))
gnss_sensor.listen(lambda event: gnss_callback(event, sensor_data))
imu_sensor.listen(lambda event: imu_callback(event, sensor_data))
lane_inv_sensor.listen(lambda event: lane_inv_callback(event, sensor_data))
obstacle_sensor.listen(lambda event: obstacle_callback(event, sensor_data, camera, K))

# Some parameters for text on screen
font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 50)
fontScale = 0.5
fontColor = (255, 255, 255)
thickness = 2
lineType = 2


# Draw the compass data (in radians) as a line with cardinal directions as capitals
def draw_compass(img, theta):
    compass_center = (700, 100)
    compass_size = 50

    cardinal_directions = [
        ('N', [0, -1]),
        ('E', [1, 0]),
        ('S', [0, 1]),
        ('W', [-1, 0])
    ]

    for car_dir in cardinal_directions:
        cv2.putText(sensor_data['rgb_image'], car_dir[0],
                    (int(compass_center[0] + 1.2 * compass_size * car_dir[1][0]), int(compass_center[1] + 1.2 * compass_size * car_dir[1][1])),
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)

    compass_point = (int(compass_center[0] + compass_size * math.sin(theta)), int(compass_center[1] - compass_size * math.cos(theta)))
    cv2.line(img, compass_center, compass_point, (255, 255, 255), 3)


# Indefinite loop
while True:

    # Latitude from GNSS sensor
    cv2.putText(sensor_data['rgb_image'], 'Lat: ' + str(sensor_data['gnss'][0]),
                (10, 30),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)

    # Longitude from GNSS sensor
    cv2.putText(sensor_data['rgb_image'], 'Long: ' + str(sensor_data['gnss'][1]),
                (10, 50),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)

    # Calculate acceleration vector minus gravity
    accel = sensor_data['imu']['accel'] - carla.Vector3D(x=0, y=0, z=9.81)

    # Display acceleration magnitude
    cv2.putText(sensor_data['rgb_image'], 'Accel: ' + str(accel),
                (10, 70),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)

    # Gyroscope output
    cv2.putText(sensor_data['rgb_image'], 'Gyro: ' + str(sensor_data['imu']['gyro']),
                (10, 100),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)

    # Compass value in radians, North is 0 radians
    cv2.putText(sensor_data['rgb_image'], 'Compass: ' + str(sensor_data['imu']['compass']),
                (10, 120),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)

    # Draw the compass
    draw_compass(sensor_data['rgb_image'], sensor_data['imu']['compass'])

    # Print 'COLLISION' to screen when flag is True
    # persist for 20 frames
    if sensor_data['collision']:
        collision_counter -= 1
        if collision_counter <= 1:
            sensor_data['collision'] = False
        cv2.putText(sensor_data['rgb_image'], 'COLLISION',
                    (250, 300),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    (255, 255, 255),
                    3,
                    2)
    else:
        collision_counter = 20

    # Print 'LANE INVASION' to screen when flag is True
    # persist for 20 frames
    if sensor_data['lane_invasion']:
        lane_invasion_counter -= 1
        if lane_invasion_counter <= 1:
            sensor_data['lane_invasion'] = False
        cv2.putText(sensor_data['rgb_image'], 'LANE INVASION',
                    (190, 350),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    (255, 255, 255),
                    3,
                    2)
    else:
        lane_invasion_counter = 20

    # Display RGB image with imshow
    cv2.imshow('RGB Camera', sensor_data['rgb_image'])

    # Break the loop if the user presses the Q key
    if cv2.waitKey(1) == ord('q'):
        break

# Close the OpenCV display window when the game loop stops and stop sensors
camera.stop()
collision_sensor.stop()
gnss_sensor.stop()
imu_sensor.stop()
lane_inv_sensor.stop()
obstacle_sensor.stop()
cv2.destroyAllWindows()
