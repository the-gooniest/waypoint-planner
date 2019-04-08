import math
import numpy as np
import pygame as pg

class Animator:

    def __init__(self, planner):
        pg.init()
        self.planner = planner
        self.height = 360
        self.width = 640
        self.screen = pg.display.set_mode((self.width, self.height))
        self.BLACK = (0,0,0)
        self.RED = (255,0,0)
        self.BLUE = (0,0,255)
        self.WHITE = (255,255,255)
        self.GRAY = (128,128,128)

    def pos_to_screen(self, position):
        """Transforms position to screen coordinates"""
        scale = 10
        new_position = np.array(position)
        new_position += self.position_offset
        new_position *= scale
        new_position = np.array([new_position[0] + self.width / 2, (self.height / 2) - new_position[1]])
        new_position = new_position.astype(int)
        return new_position

    def calculate_position_offset(self):
        self.position_offset = -self.planner.car_position
        """
        if len(self.planner.position_history) == 0:
            self.position_offset = self.planner.car_position
        else:
            offset = np.array([0.0, 0.0])
            for position in self.planner.position_history:
                offset += np.array(position)
            offset /= len(self.planner.position_history)
            self.position_offset = -1 * offset
        """

    def draw_waypoint(self, waypoint):
        """draws a waypoint"""
        waypoint = self.pos_to_screen(waypoint)
        radius = 5
        pg.draw.circle(self.screen, self.RED, waypoint, radius, 0)

    def draw_waypoints(self):
        """draws waypoints from a list of points"""
        for waypoint in self.planner.critical_waypoints:
            self.draw_waypoint(waypoint)

    def draw_trail(self):
        """draws where the car has been"""
        for position in self.planner.position_history:
            radius = 3
            center = self.pos_to_screen(position)
            pg.draw.circle(self.screen, self.GRAY, center, radius, 0)

    def draw_car(self):
        """draws a car avatar"""

        position = self.pos_to_screen(self.planner.car_position)
        heading = self.planner.car_heading

        # draw car body
        radius = 10
        pg.draw.circle(self.screen, self.BLACK, position, radius, 0)

        # indicate car's direction with line
        direction = [radius * math.cos(heading), radius * math.sin(heading)]
        direction[1] *= -1
        direction += position
        direction = direction.astype(int)
        pg.draw.line(self.screen, self.WHITE, position, direction, 2)

    def clear(self):
        self.screen.fill(self.WHITE)

    def render(self):
        self.clear()
        self.calculate_position_offset()
        self.draw_trail()
        self.draw_car()
        self.draw_waypoints()
        pg.display.flip()