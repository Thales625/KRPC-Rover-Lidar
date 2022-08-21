import cv2
import numpy as np
from math import sin, cos, radians

class Display():
    def __init__(self, lidar_distance, interval_angle):
        self.running = True

        self.distances = []
        self.actual_angle = 0
        self.lidar_distance = lidar_distance
        self.interval_angle = interval_angle
        
        self.expand = 3
        self.border = 100
        self.width = self.lidar_distance*self.expand*2 + self.border
        self.height = self.width
        self.rover_pos = (self.width//2, self.height//2)

        self.rover_color = (255, 255, 255)
        self.rover_radius = 2
        self.target_color = (255, 0, 0)
        self.target_radius = 1
        self.line_color = (0, 255, 0)

        # Text
        self.text_font = cv2.FONT_HERSHEY_COMPLEX
        self.text_font_scale = 0.7
        self.text_thickness = 1
        self.text_margin = 10
        self.text_org = np.array([10, 50])

        self.texts = []

        
    def start_loop(self):
        while self.running:
            # Clear Screen
            self.screen = np.zeros((self.width, self.height, 3), dtype=np.uint8)

            # Draw components
            self.screen = cv2.line(self.screen, self.rover_pos, self.angle_to_pos(self.actual_angle, self.lidar_distance), self.line_color, 2)
            self.screen = cv2.circle(self.screen, self.rover_pos, self.rover_radius, self.rover_color, -1)
            self.screen = cv2.ellipse(self.screen, self.rover_pos, (self.lidar_distance*self.expand, self.lidar_distance*self.expand), 0, self.interval_angle[0], self.interval_angle[1], (0, 10, 0), 1)
            self.screen = cv2.line(self.screen, self.rover_pos, np.array(self.rover_pos) + np.array([1, 0])*self.lidar_distance*self.expand, (128, 128, 128), 1)

            # Draw Obstacles
            for i, dist in enumerate(self.distances):
                if dist <= self.lidar_distance:
                    angle = i-abs(self.interval_angle[1])
                    factor = dist / self.lidar_distance
                    color = (factor*255, 0, 255 - factor*255)
                    self.screen = cv2.circle(self.screen, self.angle_to_pos(angle, dist), self.target_radius, color, -1)

            # Text
            relative_pos = np.array([0, 0])
            for row in self.texts:
                for text in row:
                    self.screen = cv2.putText(self.screen, text['text'], self.text_org + relative_pos, self.text_font, 
                        self.text_font_scale, text['color'], self.text_thickness, cv2.LINE_AA)
                    relative_pos += np.array([cv2.getTextSize(text['text'], self.text_font, self.text_font_scale, self.text_thickness)[0][0], 0])
                
                relative_pos[0] = 0
                relative_pos += np.array([0, cv2.getTextSize('|', self.text_font, self.text_font_scale, self.text_thickness)[0][1] + self.text_margin])
   
            # Show
            cv2.imshow('Lidar', self.screen)
            if cv2.waitKey(1) == ord('q'):
                self.running = False
        cv2.destroyWindow('Lidar')       
    
    def angle_to_pos(self, angle, dist):
        return (int(self.rover_pos[0] + cos(radians(angle)) * dist * self.expand), int(self.rover_pos[1] + sin(radians(angle)) * dist * self.expand))

