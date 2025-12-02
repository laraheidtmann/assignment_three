#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import time

#Node that builds and publishes the grid
class OccupancyGridCreater(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

       

        # ROS topics
        namespace = self.get_namespace()
        grid_topic = f'{namespace}/local_grid' if namespace != '/' else '/local_grid'
        scan_topic = f'{namespace}/scan' if namespace != '/' else '/scan'

        self.resolution=0.05 #meters per cell
        self.size_m=10.0   #grid cover in meters
        self.width=int(self.size_m /self.resolution)  #cells
        self.height=int(self.size_m /self.resolution) #cells
       
        self.center_i=self.width//2
        self.center_j=self.height//2

        #initialize grid
        self.grid=np.full((self.height,self.width),-1,dtype=np.int8) 


        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.grid_pub = self.create_publisher(OccupancyGrid, grid_topic, 10)

        #Pre-create a message template
        self.grid_msg=OccupancyGrid()
        self.grid_msg.info.resolution=self.resolution
        self.grid_msg.info.width=self.width
        self.grid_msg.info.height=self.height
        self.grid_msg.header.frame_id='base_link'  #grid relative to robot base

        #origin of the grid in the base_link frame
        #robot should be in center cell (0,0)
        #set origin s.t. cell (0,0) is at (-size_m/2,-size_m/2)
        origin=Pose()
        origin.position.x=-(self.width/2.0)*self.resolution
        origin.position.y=-(self.height/2.0)*self.resolution
        origin.position.z=0.0
        #orientation defaults to (0,0,0,0) and is OK for a flat grid
        self.grid_msg.info.origin=origin

        self.get_logger().info('Occupancy grid node started!')
    
    def scan_callback(self, scan: LaserScan):
        #reset grid to unknown each scan (simple for dynamic obstacles)
        self.grid.fill(-1)

        #robots cell is at center indices
        robot_i=self.center_i
        robot_j=self.center_j   

        angle=scan.angle_min

        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue  #ignore max range readings
            if r > scan.range_max:
                r=scan.range_max

            #compute obstacle point in robot frame
            x=r*math.cos(angle)
            y=r*math.sin(angle)

            #compute cell index of the hit
            hit_i=int(x/self.resolution)+self.center_i
            hit_j=int(y/self.resolution)+self.center_j

            self._raytrace_and_update(robot_i,robot_j,hit_i,hit_j)

            angle += scan.angle_increment
        #print grid for debugging
        #self.print_grid()
        
        #publish grid
        self.grid_msg.header.stamp=self.get_clock().now().to_msg()
        #flatten grid to 1D array row-major
        self.grid_msg.data=self.grid.flatten().tolist()
        self.grid_pub.publish(self.grid_msg)

        
    def print_grid(self):
        # Map values to characters for readability
        char_map = { -1: " ?", 0: " .", 100: " X" }
        for row in self.grid:
            self.get_logger().info("".join(char_map[v] for v in row))
        self.get_logger().info("\n" + "-"*80 + "\n")

    def _raytrace_and_update(self, i0, j0, i1, j1):

        # out of bounds → ignore
        if not (0 <= i1 < self.width and 0 <= j1 < self.height):
            self.get_logger().info("Out of bounds")
            return

        di = i1 - i0
        dj = j1 - j0
        steps = max(abs(di), abs(dj))
        if steps == 0:
            return

        inc_i = di / float(steps)
        inc_j = dj / float(steps)

        i = i0
        j = j0

        for step in range(steps + 1):  # +1 to include endpoint
            ci = int(round(i))
            cj = int(round(j))

            if not (0 <= ci < self.width and 0 <= cj < self.height):
                return

            if step == steps:
                # mark hit cell as occupied
                self.grid[cj, ci] = 100
            else:
                # mark free unless already occupied
                if self.grid[cj, ci] != 100:
                    self.grid[cj, ci] = 0

            i += inc_i
            j += inc_j

        

  
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridCreater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
