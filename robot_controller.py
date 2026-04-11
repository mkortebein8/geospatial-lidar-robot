import pygame
import sys
import time
from Raspbot_Lib import Raspbot
from lidar import LidarRecording
pygame.init()

pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

joystick = joysticks[0]

joystick.init()

robot = Raspbot()

lidar_list = [LidarRecording()]

while True:
    try:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                forward_speed = -joystick.get_axis(1)
                turn_speed = joystick.get_axis(0)

                left_motor = forward_speed + turn_speed
                right_motor = forward_speed - turn_speed
                
                robot.Ctrl_Car(0, 0, int(30*left_motor))
                robot.Ctrl_Car(1, 0, int(30*left_motor))
        
                robot.Ctrl_Car(2, 0, int(30*right_motor))
                robot.Ctrl_Car(3, 0, int(30*right_motor))

                print("Left Motor: ", int(30*left_motor))
                print("Right Motor: ", int(30*right_motor))

            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 3:
                    if not lidar_list[-1].recording:
                        lidar_list.start()
                        print("LiDAR recording has started.")
                    else:
                        lidar_list[-1].stop()
                        lidar_list.append(LidarRecording())
                        print("LiDAR recording has stopped.")

    except KeyboardInterrupt as e:
        robot.Ctrl_Car(0, 0, 0)
        robot.Ctrl_Car(1, 0, 0)
        robot.Ctrl_Car(2, 0, 0)
        robot.Ctrl_Car(3, 0, 0)
        break
        
