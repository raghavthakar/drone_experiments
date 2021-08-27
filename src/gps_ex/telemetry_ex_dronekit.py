#!/usr/bin/env python3

import asyncio
from dronekit import connect
import pygame

pygame.init()

#---------------------SCREEN SETUP----------------------------------------------
X=600
Y=75
# Creating the screen.
screen=pygame.display.set_mode((X, Y))
#Setting the background colour
screen.fill((200, 40, 40))
# Title and icon
pygame.display.set_caption("Object GPS")
# Font object
font = pygame.font.Font('freesansbold.ttf', 10)
#Text content
text_content=""
# Text object
text = font.render(text_content, True, (0, 0, 0), (255, 255, 255))
# text surface object
text_rect = text.get_rect()
# set the center of the rectangular object.
text_rect.center = (X // 2, Y // 2)


drone=connect('127.0.0.1:14550', wait_ready=True)
# vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=921600) # For hardware
print("Drone connected!")

while (True):
    # gets a single event from the event queue
    event = pygame.event.wait()
    # if the 'close' button of the window is pressed
    if event.type == pygame.QUIT:
        # stops the application
        break

    try:
        # Read the text file for command
        detection_file=open("detection.txt", "r")
        detection=detection_file.readlines()
        command=detection[0] #If detected or not (True or Flase)
        object=detection[1] #Type of object detected

        if(command=="True\n"):
            print(object+"Found at Global Location: %s" % drone.location.global_frame)

            text_content=object+"Found at Global Location: %s" % drone.location.global_frame
            # Text object
            text = font.render(text_content, True, (0, 0, 0), (255, 255, 255))
            # text surface object
            text_rect = text.get_rect()
            # set the center of the rectangular object.
            text_rect.center = (X // 2, Y // 2)

            screen.fill((255, 255, 255))
            screen.blit(text, text_rect)

        detection_file.close()
    except:
        pass

    # Draws the surface object to the screen.
    pygame.display.update()
