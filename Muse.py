#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import termios, tty, sys

from common.robot import Robot

# config logging
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

def main():

    logging.debug('>>> Starting Muse : Run robot, run! <<<')
    robot = Robot()

    try:
        loop = True
        logging.debug("listening to keyboard input e:forward ; d:back ; f:right ; s:left ; space:stop ; i:ir_remote ; "b: beep)
        while loop:

            k = getch()
            logging.debug(str(k))
            if k == 'e':
                robot.forward()
            if k == 'd':
                robot.backward()
            if k == 'f':
                robot.turn(-1)
            if k == 's':
                robot.turn(1)
            if k == ' ':
                robot.brake()
            if k == 'i':
                robot.ir_remote_control()
            if k == 'b':
                robot.beep()
            if robot.robot_body["US_sensor"] != False :
                if robot.robot_body["US_sensor"].value() < int(robot.robot_config["parameters"]["default_threshold_distance"]):
                    logging.debug('object found: %s' % str(robot.robot_body["US_sensor"].value()))
                    robot.brake()
            if k == 'q':
                loop = False

    # doing a cleanup action just before program ends
    # handle ctr+c and system exit
    except (KeyboardInterrupt, SystemExit) as e:
        logging.exception("Keyboard interrupt !")

    # handle exceptions
    except Exception as e:
        logging.exception("Muse.py file Exception !")

    finally:
        teardown(robot)
        logging.shutdown()

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    ch = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch

def teardown(robot):
    robot.brake()

if __name__ == "__main__":
    main()