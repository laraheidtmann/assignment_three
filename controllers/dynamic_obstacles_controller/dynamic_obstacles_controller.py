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
        self.speed = 0.0625  # m/s (meio termo entre 0.075 e 0.05) 
        
        # Each obstacle: direction vector, start position, max range, moving forward flag
        self.obstacles_data = [
            {'obstacle': self.obstacle1, 'dir': [0, 1], 'start': [-0.5, -0.5], 'range': 0.75, 'forward': True},   # Y axis
            {'obstacle': self.obstacle2, 'dir': [1, 0], 'start': [1.0, 0.0], 'range': 0.6, 'forward': True},    # X axis
            {'obstacle': self.obstacle3, 'dir': [1, 0], 'start': [0.0, 1.5], 'range': 0.675, 'forward': True},   # X axis
            {'obstacle': self.obstacle4, 'dir': [0, 1], 'start': [-1.0, 0.5], 'range': 0.75, 'forward': True},   # Y axis
            {'obstacle': self.obstacle5, 'dir': [1, 0], 'start': [1.6, -1.5], 'range': 0.7, 'forward': True}    # X axis
        ]
        
        print("Dynamic Obstacles Controller initialized")
        print(f"  - timestep: {self.timestep}ms")
        if self.obstacle1:
            print("  - DYNAMIC_OBS_1 found")
        else:
            print("  - DYNAMIC_OBS_1 NOT FOUND")
        if self.obstacle2:
            print("  - DYNAMIC_OBS_2 found")
        else:
            print("  - DYNAMIC_OBS_2 NOT FOUND")
        if self.obstacle3:
            print("  - DYNAMIC_OBS_3 found")
        else:
            print("  - DYNAMIC_OBS_3 NOT FOUND")
        if self.obstacle4:
            print("  - DYNAMIC_OBS_4 found")
        else:
            print("  - DYNAMIC_OBS_4 NOT FOUND")
        if self.obstacle5:
            print("  - DYNAMIC_OBS_5 found")
        else:
            print("  - DYNAMIC_OBS_5 NOT FOUND")
        sys.stdout.flush()
    
    def move_obstacle_oscillate(self, obs_data):
        """Move obstacle back and forth - vai e vem"""
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
        
        # Calculate movement axis (0 for X, 1 for Y)
        axis = 0 if direction[0] != 0 else 1
        
        # Define exact limit positions
        min_pos = start_pos[axis] - max_range / 2
        max_pos = start_pos[axis] + max_range / 2
        
        # Move in current direction
        dt = self.timestep / 1000.0
        if forward:
            pos[axis] += direction[axis] * self.speed * dt
            
            # Check if reached or exceeded forward limit
            if pos[axis] >= max_pos:
                pos[axis] = max_pos  # Clamp to exact position
                obs_data['forward'] = False
                self.rotate_obstacle_180(rot_field, axis)
                print(f"Obstacle at {max_pos:.2f} - turning 180° - going BACK")
        else:
            pos[axis] -= direction[axis] * self.speed * dt
            
            # Check if reached or exceeded backward limit
            if pos[axis] <= min_pos:
                pos[axis] = min_pos  # Clamp to exact position
                obs_data['forward'] = True
                self.rotate_obstacle_180(rot_field, axis)
                print(f"Obstacle at {min_pos:.2f} - turning 180° - going FORWARD")
        
        trans_field.setSFVec3f(pos)
    
    def rotate_obstacle_180(self, rot_field, axis):
        """Rotate obstacle 180 degrees around Z axis"""
        current_rot = rot_field.getSFRotation()
        # Rotation format: [x, y, z, angle]
        # Rotate 180° around Z axis: [0, 0, 1, π]
        new_angle = (current_rot[3] + math.pi) % (2 * math.pi)
        rot_field.setSFRotation([0, 0, 1, new_angle])
    
    def run(self):
        print("Starting main control loop...")
        sys.stdout.flush()
        while self.supervisor.step(self.timestep) != -1:
            # Move each obstacle with oscillating motion
            for obs_data in self.obstacles_data:
                self.move_obstacle_oscillate(obs_data)

if __name__ == "__main__":
    controller = DynamicObstaclesController()
    controller.run()
