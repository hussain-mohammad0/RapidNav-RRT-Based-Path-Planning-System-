#! /usr/bin/env python3

import numpy as np
import cv2 as cv
import os
import math
import random
import matplotlib.pyplot as plt
from PIL import Image

class rrtNavigator:
    def __init__(self) -> None:
        pass

    def calculate_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def sample_free_location(self, map_array):
        while True:
            x = random.randint(0, map_array.shape[1] - 1)
            y = random.randint(0, map_array.shape[0] - 1)
            if map_array[y, x] == 1:
                return (y, x)

    def plot_map(self, map_array):
        plt.imshow(map_array, cmap='gray')
        plt.title("Map")
        plt.axis('on')

    def find_nearest_vertex(self, vertex_set, sample):
        return min(vertex_set, key=lambda vertex: self.calculate_distance(vertex, sample))

    def steer(self, from_node, to_node, max_dist=10):
        if self.calculate_distance(from_node, to_node) < max_dist:
            return to_node
        else:
            from_node_np = np.array(from_node)
            to_node_np = np.array(to_node)
            direction = (to_node_np - from_node_np) / np.linalg.norm(to_node_np - from_node_np)
            new_point = from_node_np + direction * max_dist
            return tuple(new_point.astype(int))

    def is_obstacle_free(self, map_array, point1, point2):
        x1, y1 = point1[1], point1[0]
        x2, y2 = point2[1], point2[0]

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        for _ in range(max(dx, dy) + 1):
            if map_array[y1, x1] == 0:
                return False

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            else:
                if e2 < dx:
                    err += dx
                    y1 += sy

        return True

    def get_path_to_goal(self, map_array, start_point, goal_point, vertices, edges):
        while True:
            random_location = self.sample_free_location(map_array)
            nearest_vertex = self.find_nearest_vertex(vertices, random_location)
            new_point = self.steer(nearest_vertex, random_location, max_dist=20)
            if self.is_obstacle_free(map_array, nearest_vertex, new_point):
                vertices.add(new_point)
                edges.add((nearest_vertex, new_point))
                if self.calculate_distance(new_point, goal_point) < 20:
                    if self.is_obstacle_free(map_array, new_point, goal_point):
                        path_to_goal = self.extend_path(map_array, vertices, edges, new_point, goal_point)
                        if path_to_goal:
                            vertices.update(path_to_goal[:-1])
                            edges.update(zip(path_to_goal[:-1], path_to_goal[1:]))
                            vertices.add(goal_point)
                            return path_to_goal

    def get_path(self, map_array, startX: int, startY: int, goalX: int, goalY: int) -> list:
        start_point = (startX, startY)
        goal_point = (goalX, goalY)
        vertices, edges = set(), set()
        vertices.add(start_point)
        path_to_goal = self.get_path_to_goal(map_array, start_point, goal_point, vertices, edges)

        if path_to_goal:
            path = []
            current_vertex = goal_point
            while current_vertex != start_point:
                path.append(list(current_vertex))
                current_vertex = next((edge[0] for edge in edges if edge[1] == current_vertex), None)
                if current_vertex is None:
                    raise Exception("Path failed")
            path.append(list(start_point))
            path.reverse()

            tolerance = 10
            if self.calculate_distance(path[0], start_point) <= tolerance:
                print("Path found!")
                return path
            else:
                raise ValueError("Path does not start at the correct point. Start of path: {}, Expected: {}".format(path[0], start_point))
        else:
            print("Path not found.")
            raise Exception("Path not found")

    def extend_path(self, map_array, vertices, edges, start_node, goal):
        path = [start_node]
        current_node = start_node
        while self.calculate_distance(current_node, goal) > 20:
            random_point = goal
            nearest_vertex = self.find_nearest_vertex(vertices, random_point)
            new_point = self.steer(nearest_vertex, random_point, max_dist=20)
            if self.is_obstacle_free(map_array, nearest_vertex, new_point):
                path.append(new_point)
                current_node = new_point
            else:
                return None
        path.append(goal)
        return path

if __name__ == "__main__":
    image_path = os.path.join('C:\\Users\\Venkat Maram\\Desktop\\pa4-rrt-hussain-mohammad0\\maps', 'map2.png')
    map_image = Image.open(image_path).convert('L')
    map_array = np.array(map_image)
    map_array = np.where(map_array < 128, 0, 1)
    navigator = rrtNavigator()


