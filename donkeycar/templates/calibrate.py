#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive)


Options:
    -h --help          Show this screen.
"""
import os
import time

from docopt import docopt

import donkeycar as dk

#import parts
from donkeycar.parts.controller import LocalWebController, \
    JoystickController, WebFpv
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.utils import *

from socket import gethostname

def drive(cfg ):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    #Initialize car
    V = dk.vehicle.Vehicle()

    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT)
    V.add(ctr,
          inputs=['cam/image_array', 'tub/num_records'],
          outputs=['angle', 'throttle', 'user/mode', 'recording'],
          threaded=True)

    #this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['throttle'], outputs=['throttle'])

    drive_train = None

    #Drive train setup
    if cfg.DONKEY_GYM or cfg.DRIVE_TRAIN_TYPE == "MOCK":
        pass

    elif cfg.DRIVE_TRAIN_TYPE == "SERVO_ESC":

        from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMRearSteering, PWMThrottle, PWMBrake

        fsteering_controller = PCA9685(cfg.FRONT_STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        front_steering = PWMSteering(controller=fsteering_controller,
                                        left_pulse=cfg.FRONT_STEERING_LEFT_PWM,
                                        right_pulse=cfg.FRONT_STEERING_RIGHT_PWM)

        rsteering_controller = PCA9685(cfg.REAR_STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        rear_steering = PWMRearSteering(controller=rsteering_controller,
                                        left_pulse=cfg.REAR_STEERING_LEFT_PWM,
                                        right_pulse=cfg.REAR_STEERING_RIGHT_PWM)

        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        throttle = PWMThrottle(controller=throttle_controller,
                                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                        zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                                        min_pulse=cfg.THROTTLE_REVERSE_PWM)

        brake_controller = PCA9685(cfg.BRAKE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        brake = PWMBrake(controller=brake_controller,
                                        engaged_pulse=cfg.BRAKE_ENGAGED_PWM,
                                        released_pulse=cfg.BRAKE_RELEASED_PWM)


        drive_train = dict()
        drive_train['steering'] = front_steering
        drive_train['steering', 'throttle'] = rear_steering
        drive_train['throttle'] = throttle
        drive_train['throttle'] = brake

        V.add(front_steering, inputs=['angle'], threaded=True)
        V.add(rear_steering, inputs=['angle', 'throttle'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)
        V.add(brake, inputs=['throttle'], threaded=True)

      

    elif cfg.DRIVE_TRAIN_TYPE == "MM1":
        from donkeycar.parts.robohat import RoboHATDriver
        drive_train = RoboHATDriver(cfg)
        V.add(drive_train, inputs=['angle', 'throttle'])


    ctr.drive_train = drive_train
    ctr.drive_train_type = cfg.DRIVE_TRAIN_TYPE
    
    class ShowHowTo:
        def __init__(self):
            print(f"Go to http://{gethostname()}.local:{ctr.port}/calibrate to calibrate ")
            
        def run(self):
            pass
        
    V.add(ShowHowTo())

    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        drive(cfg)
