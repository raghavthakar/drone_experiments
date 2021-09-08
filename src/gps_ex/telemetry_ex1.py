#!/usr/bin/env python3

import asyncio
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
text_content="bruh  "
# Text object
text = font.render(text_content, True, (0, 0, 0), (255, 255, 255))
# text surface object
text_rect = text.get_rect()
# set the center of the rectangular object.
text_rect.center = (X // 2, Y // 2)

while True:
    text_content="bruh"
    # Text object
    text = font.render(text_content, True, (0, 0, 0), (255, 255, 255))
    # text surface object
    text_rect = text.get_rect()
    # set the center of the rectangular object.
    text_rect.center = (X // 2, Y // 2)

    screen.fill((255, 255, 255))
    screen.blit(text, text_rect)

    # Draws the surface object to the screen.
    pygame.display.update()
