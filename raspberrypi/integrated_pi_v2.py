# BEE 526 A
# 3/9/19
import sys
import serial, time
import curses
import os
import subprocess
from pan_tilt_v2 import CameraPanTilt

# ---- START PiCamera script setup ----
import picamera
from time import sleep
import json

CONFIGURATION_FILENAME = 'pi_camera.json'
SNAPSHOT_DIRECTORY = 'pi_camera_snapshots'
SNAPSHOT_PREFIX = 'snapshot'
MAX_RESOLUTION = (2591, 1944)
PREVIEW_RESOLUTION = (640, 480)
# ---- END PiCamera script setup ----


# set_motor_speeds()
# 
# Translates the -255 to 255 for each motor to the UART connection.
# 
# -255 <= left_motor <= 255
# -255 <= right_motor <= 255
def set_motor_speeds(serial_connection, left_motor, right_motor):
    left_motor_forward   = 1 if left_motor  < 0 else 0
    left_motor_backward  = 0 if left_motor  < 0 else 1
    right_motor_forward  = 1 if right_motor < 0 else 0
    right_motor_backward = 0 if right_motor < 0 else 1
    left_motor_speed     = int(abs(left_motor))
    right_motor_speed    = int(abs(right_motor))
    
    if (left_motor == 0 and right_motor == 0):
        left_motor_forward   = 0
        left_motor_backward  = 0
        right_motor_forward  = 0
        right_motor_backward = 0
    
    serial_connection.write(
        bytearray((left_motor_forward,
                   left_motor_backward,
                   right_motor_forward,
                   right_motor_backward,
                   left_motor_speed,
                   right_motor_speed)))
    
def stop_motors(serial_connection):
    set_motor_speeds(serial_connection, 0, 0)

def move_forward(serial_connection):
    set_motor_speeds(serial_connection, 230, 230)
    
def move_backward(serial_connection):
    set_motor_speeds(serial_connection, -230, -230)
    
def turn_left(serial_connection):
    set_motor_speeds(serial_connection, 110, -110)
    
def turn_right(serial_connection):
    set_motor_speeds(serial_connection, -110,  110)


# Boot up the camera streaming server.
subprocess.check_call(["./start_streaming.sh"], shell=True)
    
    
# Set up curses stuff
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,0,"Drive the robot!\n\nUse WASD to drive, and Space or Enter to stop.\n\nUse the arrow keys steer the camera.\n\nView the robot camera stream at http://[Robot Local IP]/native-peerconnection/.\n\nPress P to take a picture. Pictures are saved to the pi_camera_snapshots directory. Please note that taking a picture will disconnect the camera stream.\n\nPress Q to quit.\n")
stdscr.refresh()

def main():
    SERIALPORT = "/dev/ttyUSB0"
    BAUDRATE = 9600
    TIMEOUT_SECONDS = 5
    serial_connection = serial.Serial(port=SERIALPORT, 
                                      baudrate=BAUDRATE, 
                                      timeout=TIMEOUT_SECONDS)
    cam = CameraPanTilt()
    
    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.addch(20,25,key)
        stdscr.refresh()
        if key == ord('w'):
            #stdscr.addch(10,15, key)
            move_forward(serial_connection)
        elif key == ord('s'):
            #stdscr.addch(20,15, key)
            move_backward(serial_connection)
        elif key == ord('a'):
            #stdscr.addch(15,10, key)
            turn_left(serial_connection)
        elif key == ord('d'):
            #stdscr.addch(15,20, key)
            turn_right(serial_connection)
        elif key == curses.KEY_ENTER or key == curses.KEY_BACKSPACE or key == ord(' '): #
            #stdscr.addch(15,15, '_')
            stop_motors(serial_connection)
        elif key == curses.KEY_UP:
            #stdscr.addch(10,15, key)
            cam.nudge_up()
        elif key == curses.KEY_DOWN:
            #stdscr.addch(20,15, key)
            cam.nudge_down()
        elif key == curses.KEY_LEFT:
            #stdscr.addch(15,10, key)
            cam.nudge_left()
        elif key == curses.KEY_RIGHT:
            #stdscr.addch(15,20, key)
            cam.nudge_right()
        elif key == ord('p'):
            subprocess.check_call(["./stop_streaming.sh"], shell=True)
            with open(CONFIGURATION_FILENAME, 'r+') as f:
                try:
                    # read in the number of existing snapshots
                    saved_data = json.load(f)
                except:
                    # no previous file
                    saved_data = {}
                    saved_data.setdefault('num_snapshots', 0)
                
                camera = picamera.PiCamera()
                #camera.resolution = PREVIEW_RESOLUTION
                #camera.start_preview()
                #sleep(5) # hang for preview for 5 seconds
                next_snapshot_filename = SNAPSHOT_PREFIX + '_' + str(saved_data['num_snapshots']) + '.jpg'
                camera.resolution = MAX_RESOLUTION
                camera.capture(SNAPSHOT_DIRECTORY + '/' + next_snapshot_filename)
                #camera.stop_preview()
                
                saved_data['num_snapshots'] += 1
                
                # save updated data
                f.truncate(0)
                f.seek(0)
                json.dump(saved_data, f, sort_keys=True, indent=4)
            subprocess.check_call(["./start_streaming.sh"], shell=True)
        time.sleep(0.1)
        
    stop_motors(serial_connection)
    time.sleep(0.1)
    curses.endwin()



if __name__ == '__main__':
    main()