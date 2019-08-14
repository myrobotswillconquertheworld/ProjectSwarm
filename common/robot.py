# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, Motor
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor, InfraredSensor, SoundSensor, LightSensor, Sensor
import logging
import time
import yaml
import socket

class Robot:

    # Default robot speed
    DEFAULT_SPEED = 600
    # default sleep timeout in sec
    DEFAULT_SLEEP_TIMEOUT_IN_SEC = 0.1
    # default threshold distance
    DEFAULT_THRESHOLD_DISTANCE = 150
    
    # main default empty config. Sizes are in milimeters (mm). Config will load from config.yaml
    robot_name = socket.gethostname()
    robot_config = {
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
    # this variable will contain all motor and sensor objects detected during the setup process
    robot_body = {
                    "right_motor" : False,
                    "left_motor" : False,
                    "turret_rotation" : False,
                    "turret_elevation" : False,
                    "touch_sensor" : False,
                    "sound_sensor" : False,
                    "color_sensor" :False,
                    "IR_sensor" : False,
                    "US_sensor" : False,
                    "gyro_sensor" : False,
                    "other_sensor" : False
                }

    def __init__(self):
        """
        Initializing function :
        - load config
        - set up the motors
        - set up the sensors
        - store motors and sensors in robot_body
    
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

        self.load_config()
        logging.info("Load config of %s" % str(self.robot_name))

        # setting up base motors
        try:
            self.robot_body["right_motor"] = LargeMotor(OUTPUT_A)
            logging.info("Right motor on OUTPUT_A : %s" % str(self.robot_body["right_motor"].address))
            self.robot_body["right_motor"].reset()
        except:
            logging.error("No Large Motor detected on Output A (right wheel)")
        
        try:
            self.robot_body["left_motor"] = LargeMotor(OUTPUT_B)
            logging.info("Left motor on OUTPUT_B: %s" % str(self.robot_body["left_motor"].address))
            self.robot_body["left_motor"].reset()
        except:
            logging.error("No Large Motor detected on Output B (left wheel)")
            
        try:
            self.robot_body["turret_rotation"] = MediumMotor(OUTPUT_C)
            logging.info("Turret rotation motor on OUTPUT_C: %s" % str(self.robot_body["turret_rotation"].address))
            self.robot_body["turret_rotation"].reset()
        except:
            logging.error("No Medium Motor detected on Output C (turret rotation)")  
            

        logging.info("Wheels and turret motors setup and reset done")

        # Setting up sensors
        try:
            self.robot_body["gyro_sensor"] = GyroSensor()
            logging.info("gyro sensor connected: %s" % str(self.robot_body["gyro_sensor"].address))
            self.robot_body["gyro_sensor"].mode = 'GYRO-ANG'
        except Exception as e:
            self.robot_body["gyro_sensor"] = False
            logging.exception("Gyro sensor not connected")

        try:
            self.robot_body["color_sensor"] = ColorSensor()
            logging.info("color sensor connected: %s" % str(self.robot_body["color_sensor"].address))
            self.robot_body["color_sensor"].mode = 'COL-REFLECT'
        except Exception as e:
            self.robot_body["color_sensor"] = False
            logging.exception("Color sensor not connected")

        try:
            self.robot_body["ultrasonic_sensor"] = UltrasonicSensor()
            logging.info("ultrasonic sensor connected: %s" % str(self.robot_body["ultrasonic_sensor"].address))
            self.robot_body["ultrasonic_sensor"].mode = 'US-DIST-CM'
        except Exception as e:
            self.robot_body["ultrasonic_sensor"] = False
            logging.exception("Ultrasonic sensor not connected")

        try:
            self.robot_body["ir_sensor"] = InfraredSensor()
            logging.info("ir sensor connected: %s" % str(self.robot_body["ir_sensor"].address))
            self.robot_body["ir_sensor"].mode = 'IR-REMOTE'
        except Exception as e:
            self.robot_body["ir_sensor"] = False
            logging.exception("IR sensor not connected")
            
    def load_config(self):
        """Read config file and load it in class"""
        with open('common/config.yaml') as f:
            swarm_config = yaml.load(f, Loader=yaml.FullLoader)
            logging.info("loading config for: %s" % self.robot_name)
            self.robot_config = swarm_config[self.robot_name]
            print(self.robot_config)
            #self.robot_config = self.robot_config[self.robot_name] 
        
        return True
    

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
            
        if self.robot_body["right_motor"] != False and self.robot_body["left_motor"] != False :
            self.robot_body["right_motor"].run_forever()
            self.robot_body["left_motor"].run_forever()
        else :
            logging.error("At least one wheel motor is unavailable. Cannot move forward.")

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

        if self.robot_body["right_motor"] != False and self.robot_body["left_motor"] != False :
            self.robot_body["right_motor"].run_forever()
            self.robot_body["left_motor"].run_forever()
        else :
            logging.error("At least one wheel motor is unavailable. Cannot move back.")

    def brake(self):
        """Stops all motors of the robot"""
        
        if self.robot_body["right_motor"] != False :
            self.robot_body["right_motor"].stop()
        else :
            logging.info("No right motor available to stop")
            
        if self.robot_body["left_motor"] != False :
            self.robot_body["left_motor"].stop()
        else :
            logging.error("No left motor available to stop.")

    def turn(self, right_or_left=1):
        """
        Turn the robot at default speed indefinitely
    
        Args :
            right_or_left=1 turn right if positive, left if negative
            
        Returns :
            False if one wheel motor is missin
            True otherwise
        
        To do :
            add speed args
            protect right or left arg (reject other than -1 or 1)
        """
        if self.robot_body["right_motor"] != False and self.robot_body["left_motor"] != False :
            logging.error("At least one motor missing, cannot turn")
            return False

        logging.debug("Turning !!")

        self.set_speed(self.DEFAULT_SPEED)

        self.robot_body["right_motor"].speed_sp *= right_or_left
        self.robot_body["left_motor"].speed_sp *= -right_or_left
        self.robot_body["right_motor"].run_forever()
        self.robot_body["left_motor"].run_forever()
        
        return True

    def set_speed(self, speed):
        """
        Set speed on both propulsion motors
    
        Args :
            speed : integer (no unit)
        
        Returns :
            False if one wheel motor is missin
            True otherwise   
        """
        if self.robot_body["right_motor"] != False :
            self.robot_body["right_motor"].speed_sp = speed
        else :
            return False
              
        if self.robot_body["right_motor"] != False :
            self.robot_body["left_motor"].speed_sp = -speed
        else :
            return False
        
        return True


    def ir_remote_control(self):
        """
        Handles IR remote control inputs and moves the robot accordingly. 
        Stops robot if ultrasonic sensor gets a value below DEFAULT_THRESHOLD_DISTANCE.
        IR values :
        * 0 -> brake
        * 1 -> forward
        * 2 -> turn left
        * 3 -> backward
        * 4 -> turn right
        * 5 -> exit function
        * 9 -> brake
    
        Returns :
            True if IR value is 5 (exit)
        """
        while True:
            time.sleep(self.DEFAULT_SLEEP_TIMEOUT_IN_SEC)
            ir_value = self.ir_sensor.value()

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
            if self.robot_body["ultrasonic_sensor"] != False :
                if self.robot_body["ultrasonic_sensor"].value() < self.DEFAULT_THRESHOLD_DISTANCE:
                    logging.debug('object found: %s' % str(self.ultrasonic_sensor.value()))
                    self.brake()


