# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, Motor
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor, InfraredSensor, SoundSensor, LightSensor, Sensor
import logging
import time
import yaml

class Robot:

    # Default robot speed
    DEFAULT_SPEED = 600
    # default sleep timeout in sec
    DEFAULT_SLEEP_TIMEOUT_IN_SEC = 0.1
    # default threshold distance
    DEFAULT_THRESHOLD_DISTANCE = 150
    
    # main default empty config. Sizes are in milimeters (mm)
    robot_config = {
                    "Name" : None,
                    "A" : False,
                    "B" : False,
                    "C" : False,
                    "D" : False,
                    "S1" : False,
                    "S2" : False,
                    "S3" : False,
                    "S4" : False,
                    "width" : 0,
                    "height" : 0,
                    "length" : 0
                    }

    
    # sensor list
    sensorsList = [FALSE, FALSE, FALSE, FALSE]
    
    # motors array
    motorsList = [FALSE, FALSE, FALSE, FALSE]

    def __init__(self):
        """
        Initializing function :
        - load config
        - set up the motors
        - set up the sensors
    
        Args :
            self
        
        Returns :
            nothing
    
        Raises :
            Gyro sensor not detected
            Color sensor not detected
            Ultrasonic sensor not detected
            IR sensor not detected
        
        To do :
            debug sensor detection
            handle motor not responding or not connected
            add turret small motor
        """

        logging.debug("Setting up...")
        name = self.load_config()
        logging.debut("Load config of " % str(name))

        # setting up base motors
        right_motor = LargeMotor(OUTPUT_A)
        logging.info("Right motor on OUTPUT_A : %s" % str(right_motor.address))

        left_motor = LargeMotor(OUTPUT_B)
        logging.info("Left motor on OUTPUT_B: %s" % str(left_motor.address))

        right_motor.reset()
        left_motor.reset()
        logging.info("motor reset done")
        self.motors = [left_motor, right_motor]

        self.right_motor = right_motor
        self.left_motor = left_motor

        # Setting up sensors
        try:
            gyro_sensor = GyroSensor()
            logging.info("gyro sensor connected: %s" % str(gyro_sensor.address))
            gyro_sensor.mode = 'GYRO-ANG'
            self.gyro_sensor = gyro_sensor
        except Exception as e:
            logging.exception("Gyro sensor not connected")

        try:
            color_sensor = ColorSensor()
            logging.info("color sensor connected: %s" % str(color_sensor.address))
            color_sensor.mode = 'COL-REFLECT'
            self.color_sensor = color_sensor
        except Exception as e:
            logging.exception("Color sensor not connected")

        try:
            ultrasonic_sensor = UltrasonicSensor()
            logging.info("ultrasonic sensor connected: %s" % str(ultrasonic_sensor.address))
            ultrasonic_sensor.mode = 'US-DIST-CM'
            self.ultrasonic_sensor = ultrasonic_sensor
        except Exception as e:
            logging.exception("Ultrasonic sensor not connected")

        try:
            ir_sensor = InfraredSensor()
            logging.info("ir sensor connected: %s" % str(ir_sensor.address))
            ir_sensor.mode = 'IR-REMOTE'
            self.ir_sensor = ir_sensor
        except Exception as e:
            logging.exception("IR sensor not connected")
            
    def load_config(self):
        """Read config file and load it in class"""
        with open('config.yaml') as f:
            self.robot_config = yaml.load(f, Loader=yaml.FullLoader)
        
        return self.robot_config["Name"]
    

    def forward(self, speed=None):
        """
        This function moves the robot forward indefinitely.
        
        Args :
            speed (unit not defined for now)
    
        To do :
            add unit for speed
            handle stalled motor
        """
        if speed:
            self.set_speed(abs(speed))
        else:
            self.set_speed(abs(self.DEFAULT_SPEED))

        self.right_motor.run_forever()
        self.left_motor.run_forever()

    def backward(self, speed=None):
        """
        This function moves the robot backwards indefinitely. It convert speed to a negative number.
    
        Args :
            speed (unit not defined for now)
    
        To Do :
            delete function (replace by forward with negative speed)
        """

        if speed:
            self.set_speed(-abs(speed))
        else:
            self.set_speed(-abs(self.DEFAULT_SPEED))

        self.right_motor.run_forever()
        self.left_motor.run_forever()

    def brake(self):
        """Stops all motors of the robot"""
        
        for m in self.motors:
            m.stop()

    def turn(self, right_or_left=1):
        """
        Turn the robot at default speed indefinitely
    
        Args :
            right_or_left=1 turn right if positive, left if negative
        
        To do :
            add speed args
            protect right or left arg (reject other than -1 or 1)
        """

        logging.debug("Turning !!")

        self.set_speed(self.DEFAULT_SPEED)

        self.right_motor.speed_sp *= right_or_left
        self.left_motor.speed_sp *= -right_or_left
        self.right_motor.run_forever()
        self.left_motor.run_forever()

    def set_speed(self, speed):
        """
        Set speed on both propulsion motors
    
        Args :
            speed : integer (no unit)
        """
        self.right_motor.speed_sp = speed
        self.left_motor.speed_sp = -speed

    def ir_remote_control(self):
        """
        Handles IR remote control inputs and moves the robot accordingly. Stops robot if ultrasonic sensor gets a value below DEFAULT_THRESHOLD_DISTANCE.
        IR values :
        0 -> brake
        1 -> forward
        2 -> turn left
        3 -> backward
        4 -> turn right
        5 -> exit function
        9 -> brake
    
        Returns :
            True if IR value is 5 (exit)
        
        To do :
            Handle error when ultrasonic sensor not available.
        """
        while True:
            time.sleep(self.DEFAULT_SLEEP_TIMEOUT_IN_SEC)
            ir_value = self.ir_sensor.value()
            #logging.debug('ir value: %s' % str(ir_value))

            if ir_value == 0:
                self.brake()
            elif ir_value == 1:
                self.forward(self.DEFAULT_SPEED)
                #logging.debug('button top left is pressed' % top_left(channel=1))
            elif ir_value == 2:
                self.turn(-1)
            elif ir_value == 3:
                self.backward(self.DEFAULT_SPEED)
            elif ir_value == 4:
                self.turn(1)
            elif ir_value == 5:
                return True
            elif ir_value == 9:
                self.brake()
            if self.ultrasonic_sensor.value() < self.DEFAULT_THRESHOLD_DISTANCE:
                logging.debug('object found: %s' % str(self.ultrasonic_sensor.value()))
                self.brake()


