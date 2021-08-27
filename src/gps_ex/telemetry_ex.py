#!/usr/bin/env python3

import asyncio
from mavsdk import System
import pygame

pygame.init()

#---------------------SCREEN SETUP----------------------------------------------
X=1000
Y=1000
# Creating the screen.
screen=pygame.display.set_mode((X, Y))
#Setting the background colour
screen.fill((200, 40, 40))
# Title and icon
pygame.display.set_caption("Object GPS")
# Font object
font = pygame.font.Font('freesansbold.ttf', 32)
#Text content
text_content=""
# Text object
text = font.render(text_content, True, (0, 0, 0), (255, 255, 255))
# text surface object
text_rect = text.get_rect()
# set the center of the rectangular object.
text_rect.center = (X // 2, Y // 2)

async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Start the tasks
    # asyncio.ensure_future(print_battery(drone))
    # asyncio.ensure_future(print_gps_info(drone))
    # asyncio.ensure_future(print_in_air(drone))
    asyncio.ensure_future(print_position(drone))

async def print_position(drone):
    global command_file
    async for position in drone.telemetry.position():

        detection_file=open("detection.txt", "r")
        detection=detection_file.readlines()
        command=detection[0]
        object=detection[1]

        if(command=="True\n"):
            text_content=object+" found!: "+str(position.latitude_deg)+', '+str(position.longitude_deg)
            # Text object
            text = font.render(text_content, True, (0, 0, 0), (255, 255, 255))
            # text surface object
            text_rect = text.get_rect()
            # set the center of the rectangular object.
            text_rect.center = (X // 2, Y // 2)

            screen.fill((255, 255, 255))
            screen.blit(text, text_rect)

        detection_file.close()

        # Draws the surface object to the screen.
        pygame.display.update()

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
