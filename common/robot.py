# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, Motor
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor, InfraredSensor, SoundSensor, LightSensor, Sensor
import logging
import time
import yaml
import socket
import ev3dev2.Sound

class Robot:

    # default sleep timeout in sec
    DEFAULT_SLEEP_TIMEOUT_IN_SEC = 0.1
    
    # main default empty config. Sizes are in milimeters (mm). Config will load from config.yaml
    robot_name = socket.gethostname()
    robot_config = {
                    "motors" : {"A" : False, "B" : False, "C" : False, "D" : False},
                    "sensors" : {"S1" : False, "S2" : False, "S3" : False, "S4" : False},
                    "dimensions" : {"wheel_diameter" : 0, "wheel_spacing" : 0},
                    "parameters" : {"default_speed": 400, "threshold_distance": 100}
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
        - store motors and sensors configuration in robot_body
    
        Raises :
            Config error
            Motor error
            Sensor error
        
        To do :
            debug sensor detection
            handle motor not responding or not connected
            add turret small motor
        """
        
        # loading config form config.yaml file
        self.load_config()
        logging.info(">>> Config loaded for:" + str(self.robot_name) + "<<<")
            
        self.setup_motors()
        logging.info(">>> Motors config done <<<")
            
        self.setup_sensors()
        logging.info(">>> Sensors setup done <<<")
        
    def beep():
        Sound.beep()
        return True
            
    def load_config(self):
        """Read config file and load it in class"""
        with open('common/config.yaml') as f:
            swarm_config = yaml.load(f, Loader=yaml.FullLoader)
            self.robot_config = swarm_config[self.robot_name]
        
        return True
        
    def setup_sensors(self):
        """During startup setup all sensors of the robot based on the given config.yaml"""
        
        #if self.robot_body().has_key("")
        for inx, sensor in self.robot_config["sensors"].items():
            if sensor == "Touch_sensor":
                try:
                    self.robot_body["touch_sensor"] = TouchSensor()
                    logging.info("Touch sensor connected:" + str(self.robot_body["touch_sensor"].address)) 
                except:
                    self.robot_body["touch_sensor"] = False
                    logging.exception("Touch sensor not connected")
                    
            elif sensor == "Sound_sensor":
                try:
                    self.robot_body["sound_sensor"] = SoundSensor()
                    logging.info("Sound sensor connected: %s" % str(self.robot_body["sound_sensor"].address))
                except:
                    self.robot_body["sound_sensor"] = False
                    logging.exception("Sound sensor not connected")
                    
            elif sensor == "Gyro_sensor":
                try:
                    self.robot_body["gyro_sensor"] = GyroSensor()
                    logging.info("Gyro sensor connected: %s" % str(self.robot_body["gyro_sensor"].address))
                    self.robot_body["gyro_sensor"].mode = 'GYRO-ANG'
                except:
                    self.robot_body["gyro_sensor"] = False
                    logging.exception("Gyro sensor not connected")
                    
            elif sensor == "Color_sensor":
                try:
                    self.robot_body["color_sensor"] = ColorSensor()
                    logging.info("Color sensor connected: %s" % str(self.robot_body["color_sensor"].address))
                    self.robot_body["color_sensor"].mode = 'COL-REFLECT'
                except:
                    self.robot_body["color_sensor"] = False
                    logging.exception("Color sensor not connected")
                    
            elif sensor == "US_sensor":
                try:
                    self.robot_body["US_sensor"] = UltrasonicSensor()
                    logging.info("Ultrasonic sensor connected: %s" % str(self.robot_body["US_sensor"].address))
                    self.robot_body["US_sensor"].mode = 'US-DIST-CM'
                except:
                    self.robot_body["US_sensor"] = False
                    logging.exception("Ultrasonic sensor not connected")
                    
            elif sensor == "IR_sensor":
                try:
                    self.robot_body["IR_sensor"] = InfraredSensor()
                    logging.info("IR sensor connected: %s" % str(self.robot_body["IR_sensor"].address))
                    self.robot_body["IR_sensor"].mode = 'IR-REMOTE'
                except:
                    self.robot_body["IR_sensor"] = False
                    logging.exception("IR sensor not connected")
                    
            elif sensor == "DIST-Nx-v3":
                logging.info("DIST-Nx-v3 sensor not supported yet")
                
            elif sensor == "Compass_sensor":
                logging.info("Compass sensor not supported yet")
                    
            else:
                logging.error("Unknown sensor / not supported on input : " + str(inx))
        
        return True
        
    def setup_motors(self):
        """ Setup 2 wheel motors and 2 turret motors"""
        
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
            
        # setting up turret motors
        try:
            self.robot_body["turret_rotation"] = Motor(OUTPUT_C)
            logging.info("Turret rotation motor on OUTPUT_C: %s" % str(self.robot_body["turret_rotation"].address))
            self.robot_body["turret_rotation"].reset()
        except:
            logging.error("No Medium Motor detected on Output C (turret rotation)")  
            
        try:
            self.robot_body["turret_elevation"] = Motor(OUTPUT_D)
            logging.info("Turret elevation motor on OUTPUT_D: %s" % str(self.robot_body["turret_elevation"].address))
            self.robot_body["turret_elevation"].reset()
        except:
            logging.error("No Medium Motor detected on Output D (turret elevation)") 
        
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
            self.set_speed(-abs(speed))
        else:
            self.set_speed(-abs(int(self.robot_config["parameters"]["default_speed"])))
            
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
            self.set_speed(abs(speed))
        else:
            self.set_speed(abs(int(self.robot_config["parameters"]["default_speed"])))

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
            False if one wheel motor is missing
            True otherwise
        
        To do :
            add speed args
            protect right or left arg (reject other than -1 or 1)
        """
        if self.robot_body["right_motor"] == False or self.robot_body["left_motor"] == False :
            logging.error("At least one motor missing, cannot turn")
            return False

        logging.debug("Turning !!")

        self.set_speed(int(self.robot_config["parameters"]["default_speed"]))

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
            False if one wheel motor is missing
            True otherwise   
        """
        if self.robot_body["right_motor"] != False :
            self.robot_body["right_motor"].speed_sp = speed
        else :
            return False
              
        if self.robot_body["right_motor"] != False :
            self.robot_body["left_motor"].speed_sp = speed
        else :
            return False
        
        return True


    def ir_remote_control(self):
        """
        Handles IR remote control inputs and moves the robot accordingly. 
        Stops robot if ultrasonic sensor gets a value below default_threshold_distance (parameter from config).
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
            False if no IR sensor
        """
        if self.robot_body["IR_sensor"] == False :
            logging.info("No IR sensor, remote control operation impossible.")
            return False
        
        while True:
            time.sleep(self.DEFAULT_SLEEP_TIMEOUT_IN_SEC)
            ir_value = self.robot_body["IR_sensor"].value()

            if ir_value == 0:
                self.brake()
            elif ir_value == 1:
                self.forward()
            elif ir_value == 2:
                self.turn(-1)
            elif ir_value == 3:
                self.backward()
            elif ir_value == 4:
                self.turn(1)
            elif ir_value == 5:
                return True
            elif ir_value == 9:
                self.brake()
                
            if self.robot_body["US_sensor"] != False :
                if self.robot_body["US_sensor"].value() < int(self.robot_config["parameters"]["default_threshold_distance"]):
                    logging.debug('object found: %s' % str(self.robot_body["US_sensor"].value()))
                    self.brake()


