class Robot:
    def __init__(self):
        self.mission = None
        self.rotation_angle = 5
        self.bool_long_shot = True
        self.dist_ball = 1000

        self.yaw = 0
        self.pitch = 50
        
        self.bool_arrow = False
        self.bool_ball = False
        self.bool_shot_zone = False
        self.bool_hole = False
        self.bool_bunker = False
    
    def reset(self, mission):
        self.mission = mission
        self.rotation_angle = 5
        self.bool_long_shot = True
        self.dist_ball = 1000

        self.yaw = 0
        self.pitch = 50
        
        self.bool_arrow = False
        self.bool_ball = False
        self.bool_shot_zone = False
        self.bool_hole = False
        self.bool_bunker = False
    
    def robot_message(self):
        print(f"Mission: {self.mission}, Distance: {self.dist_ball}, Pitch: {self.pitch}, Yaw: {self.yaw}")
