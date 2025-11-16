class Obstacle():
    def __init__(self):
        self.beginning_border_dist = 0
        self.ending_border_dist = 0
        self.beginning_border_angle = 0
        self.ending_border_angle = 0
        # list of (angle_in_deg, distance) tuples
        self.lidar_points = [] 

    def get_beginning_border(self):
        return self.beginning_border_dist, self.beginning_border_angle
    
    def get_ending_border(self):
        return self.ending_border_dist, self.ending_border_angle

    def set_beginning_border(self, dist, angle):
        self.beginning_border_dist = dist
        self.beginning_border_angle = angle

    def set_ending_border(self, dist, angle):
        self.ending_border_dist = dist
        self.ending_border_angle = angle

    def calculate_obstacle_size(self):
        r1 = self.beginning_border_dist
        r2 = self.ending_border_dist
        theta1 = math.radians(self.beginning_border_angle)
        theta2 = math.radians(self.ending_border_angle)
        return math.sqrt(r1**2 + r2**2 - 2 * r1 * r2 * math.cos(theta2 - theta1))

    def min_distance(self):
        if not self.lidar_points:
            return None
        return min(d for _, d in self.lidar_points)

    # check if any point lies in a front window around angle 0
    def is_in_front(self, front_min_angle=-15, front_max_angle=15):
        for angle, _ in self.lidar_points:
            if front_min_angle <= angle <= front_max_angle:
                return True
        return False
