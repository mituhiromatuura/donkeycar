"""
CAR CONFIG

This file is read by your car application's manage.py script to change the car
performance.

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/mycar/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

#CAMERA
CAMERA_RESOLUTION = (120, 160) #(height, width)
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

#STEERING

#THROTTLE
THROTTLE_FORWARD_PWM = 2000
THROTTLE_STOPPED_PWM = 0
THROTTLE_REVERSE_PWM = -2000

#PAN
PAN_CHANNEL = 1
PAN_LEFT_PWM = 140
PAN_RIGHT_PWM = 640
PAN_DIR = 1

#TILT
TILT_CHANNEL = 0
TILT_DOWN_PWM = 140
TILT_UP_PWM = 640
TILT_DIR = 1

#TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8


#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = False
JOYSTICK_MAX_THROTTLE = 0.25
JOYSTICK_STEERING_SCALE = 0.5
AUTO_RECORD_ON_THROTTLE = False


USE_SINGLE_TUB = False
TUB_PATH = os.path.join(CAR_PATH, 'tub') # if using a single tub

#ROPE.DONKEYCAR.COM
ROPE_TOKEN="GET A TOKEN AT ROPE.DONKEYCAR.COM"
