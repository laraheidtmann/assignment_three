#!/usr/bin/env python3
"""
Webots Supervisor controller to move dynamic obstacles automatically.
No ROS2 integration needed - pure Webots controller.
"""

from controller import Supervisor
import math
import sys

class DynamicObstaclesController:
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        # Get references to dynamic obstacles
        self.obstacle1 = self.supervisor.getFromDef("DYNAMIC_OBS_1")
        self.obstacle2 = self.supervisor.getFromDef("DYNAMIC_OBS_2")
        self.obstacle3 = self.supervisor.getFromDef("DYNAMIC_OBS_3")
        self.obstacle4 = self.supervisor.getFromDef("DYNAMIC_OBS_4")
        self.obstacle5 = self.supervisor.getFromDef("DYNAMIC_OBS_5")
        
        # Movement parameters - simple back and forth
        self.speed = 0.0625  # m/s

        # Each obstacle: direction vector, start position, max range, moving forward flag
        self.obstacles_data = [
            {'obstacle': self.obstacle1, 'dir': [0, 1], 'start': [-0.5, -0.5], 'range': 0.75, 'forward': True},
            {'obstacle': self.obstacle2, 'dir': [1, 0], 'start': [1.0, 0.0], 'range': 0.6, 'forward': True},
            {'obstacle': self.obstacle3, 'dir': [1, 0], 'start': [0.0, 1.5], 'range': 0.675, 'forward': True},
            {'obstacle': self.obstacle4, 'dir': [0, 1], 'start': [-1.0, 0.5], 'range': 0.75, 'forward': True},
            {'obstacle': self.obstacle5, 'dir': [1, 0], 'start': [1.6, -1.5], 'range': 0.7, 'forward': True},
        ]
        
        print("Dynamic Obstacles Controller initialized")
        print(f"  - timestep: {self.timestep}ms")
        for i, obs in enumerate([self.obstacle1, self.obstacle2, self.obstacle3, self.obstacle4, self.obstacle5], start=1):
            status = "found" if obs else "NOT FOUND"
            print(f"  - DYNAMIC_OBS_{i} {status}")
        sys.stdout.flush()

    def move_obstacle_oscillate(self, obs_data):
        """Move obstacle back and forth"""
        obstacle = obs_data['obstacle']
        if not obstacle:
            return

        trans_field = obstacle.getField("translation")
        rot_field = obstacle.getField("rotation")

        pos = list(trans_field.getSFVec3f())
        direction = obs_data['dir']
        start_pos = obs_data['start']
        max_range = obs_data['range']
        forward = obs_data['forward']

        axis = 0 if direction[0] != 0 else 1
        min_pos = start_pos[axis] - max_range / 2
        max_pos = start_pos[axis] + max_range / 2

        dt = self.timestep / 1000.0
        if forward:
            pos[axis] += direction[axis] * self.speed * dt
            if pos[axis] >= max_pos:
                pos[axis] = max_pos
                obs_data['forward'] = False
                self.rotate_obstacle_180(rot_field)
        else:
            pos[axis] -= direction[axis] * self.speed * dt
            if pos[axis] <= min_pos:
                pos[axis] = min_pos
                obs_data['forward'] = True
                self.rotate_obstacle_180(rot_field)

        trans_field.setSFVec3f(pos)

    def rotate_obstacle_180(self, rot_field):
        current_rot = rot_field.getSFRotation()
        new_angle = (current_rot[3] + math.pi) % (2 * math.pi)
        rot_field.setSFRotation([0, 0, 1, new_angle])

    def run(self):
        print("Starting main control loop...")
        sys.stdout.flush()
        while self.supervisor.step(self.timestep) != -1:
            for obs_data in self.obstacles_data:
                self.move_obstacle_oscillate(obs_data)

if __name__ == "__main__":
    controller = DynamicObstaclesController()
    controller.run()
