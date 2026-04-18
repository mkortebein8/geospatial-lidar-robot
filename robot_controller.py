import pygame
import sys
import time
import csv
from Raspbot_Lib import Raspbot
from lidar import LidarRecording
pygame.init()

pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

joystick = joysticks[0]

joystick.init()

robot = Raspbot()

lidar_inst = LidarRecording()

lidar_done = False

recording_count = 0

while not lidar_done:
    try:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                forward_speed = -joystick.get_axis(1)
                turn_speed = joystick.get_axis(0)

                left_motor = forward_speed + turn_speed
                right_motor = forward_speed - turn_speed
                
                if left_motor >= 0:
                    robot.Ctrl_Car(0, 0, int(30*left_motor))
                    robot.Ctrl_Car(1, 0, int(30*left_motor))
                if left_motor < 0:
                    robot.Ctrl_Car(0, 1, -1 * int(30*left_motor))
                    robot.Ctrl_Car(1, 1, -1 * int(30*left_motor))
                if right_motor >= 0:
                    robot.Ctrl_Car(2, 0, int(30*right_motor))
                    robot.Ctrl_Car(3, 0, int(30*right_motor))
                if right_motor < 0:
                    robot.Ctrl_Car(2, 1, -1 * int(30*right_motor))
                    robot.Ctrl_Car(3, 1, -1 * int(30*right_motor))

                print("Left Motor: ", int(30*left_motor))
                print("Right Motor: ", int(30*right_motor))

            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 3:
                    if not lidar_inst.recording:
                        lidar_inst.start()
                        print("LiDAR recording has started.")
                    else:
                        lidar_inst.stop()
                        # Writes the LiDAR data to a csv file.
                        stuff_for_the_file = zip(lidar_inst.angle_data, lidar_inst.dist_data, lidar_inst.time_data)
                        file_name = f'lidar_record_{recording_count}.csv'
                        with open(file_name, 'w', newline='') as file:
                            writer = csv.writer(file)
                            write.writerow(['angle', 'distance', 'unix_time'])
                            writer.writerows(stuff_for_the_file)
                        
                        # Creates a new LidarRecording instance
                        lidar_inst = LidarRecording()
                        recording_count += 1
                        print("LiDAR recording has stopped.")
                if event.button == 4:
                   lidar_done = True

    except KeyboardInterrupt as e:
        robot.Ctrl_Car(0, 0, 0)
        robot.Ctrl_Car(1, 0, 0)
        robot.Ctrl_Car(2, 0, 0)
        robot.Ctrl_Car(3, 0, 0)
        break
        
