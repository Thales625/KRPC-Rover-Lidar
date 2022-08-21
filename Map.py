import cv2
from math import ceil

class Map():
    def __init__(self, body_name) -> None:
        self.running = True
        self.body_name = body_name
        self.objects = []

    def start_loop(self):
        while self.running:
            self.map = cv2.imread(f'./maps/{self.body_name}.jpg')
            self.map_size = (1024, 512)
            self.map = cv2.resize(self.map, self.map_size)

            for object in self.objects:
                self.map = cv2.circle(self.map, self.cords_to_pos(object['pos']), object['radius'], object['color'], -1)

            cv2.imshow('Map', self.map)
            if cv2.waitKey(10) == ord('q'):
                self.running = False

        cv2.destroyWindow('Map')

    def cords_to_pos(self, lng_lat):
        lng, lat = lng_lat

        x = ((lng + 180) / 360) * self.map_size[0]
        y = ((-lat + 90) / 180) * self.map_size[1]

        return (ceil(x), ceil(y))