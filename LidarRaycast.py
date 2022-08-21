import krpc
from math import ceil, radians, sin, cos
from threading import Thread
from Display import Display
from Map import Map
from WheelController import WheelController


class LidarRaycast():
    def __init__(self) -> None:
        self.conn = krpc.connect('Lidar')
        self.space_center = self.conn.space_center
        self.rover = self.space_center.active_vessel
        self.body = self.rover.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.rover.surface_reference_frame
        self.flight = self.rover.flight(self.body_ref)

        # Streams
        self.speed = self.conn.add_stream(getattr, self.flight, "horizontal_speed")
        self.longitude = self.conn.add_stream(getattr, self.flight, "longitude")
        self.latitude = self.conn.add_stream(getattr, self.flight, "latitude")

        # Get Lidar Parts
        try:
            self.lidar = self.rover.parts.with_tag('lidar')[0]
        except:
            print('Não foi possível encontrar Lidar.')
            exit()

        self.target = self.space_center.target_vessel
        if self.target == None:
            print('Selecione um alvo!')
            exit()


        self.rotation_right = True

        self.max_angle = 45 # POSITIVE <- Max 180
        self.min_angle = -self.max_angle
        self.step_angle = 3
        self.actual_angle = 0

        self.mean_angle = (abs(self.min_angle)+abs(self.max_angle)) // 2

        self.lidar_distance = 50
        self.lidar_ref = self.lidar.reference_frame

        self.distances = [self.lidar_distance + 1 for i in range(self.min_angle, self.max_angle + 1)]

        # Batery
        self.batery_limit = self.rover.resources.max('ElectricCharge')

        # DISPLAY
        self.frame = Display(self.lidar_distance, [self.min_angle, self.max_angle])
        Thread(target=self.frame.start_loop).start()
        self.frame.texts = [
            [
                {
                    'text': 'Bateria: ',
                    'color': (255, 255, 255)
                },
                {
                    'text': f'-',
                    'color': (255, 255, 255)
                }
            ],
            [
                {
                    'text': 'Distancia: ',
                    'color': (255, 255, 255)
                },
                {
                    'text': f'-',
                    'color': (255, 255, 255)
                }
            ],
            [
                {
                    'text': 'Velocidade: ',
                    'color': (255, 255, 255)
                },
                {
                    'text': f'-',
                    'color': (255, 255, 255)
                }
            ],
            [
                {
                    'text': 'Error: ',
                    'color': (255, 255, 255)
                },
                {
                    'text': f'-',
                    'color': (255, 255, 255)
                }
            ]
        ]

        # MAP
        self.map = Map(self.body.name.lower())
        Thread(target=self.map.start_loop).start()

        # WHEEL CONTROLLER
        self.target_flight = self.target.flight(self.body_ref)
        # -> Stream Lng / Lat
        self.target_lng = self.conn.add_stream(getattr, self.target_flight, "longitude")
        self.target_lat = self.conn.add_stream(getattr, self.target_flight, "latitude")

        self.controller = WheelController(self.space_center, self.rover, self.surface_ref)
        self.controller.set_target_pos(self.target.position(self.surface_ref))
        Thread(target=self.controller.start_loop).start()


        # Main Loop
        while True:
            self.actual_direction = (-sin(radians(self.actual_angle)), 0.01, -cos(radians(self.actual_angle)))
            laser_distance = min(self.lidar_distance+1, self.space_center.raycast_distance((0, .1, 0), self.actual_direction, self.lidar_ref))

            self.set_angle_value(self.actual_angle, laser_distance)

            # Rotation
            if self.actual_angle >= self.max_angle:
                self.rotation_right = False
            elif self.actual_angle <= self.min_angle:
                self.rotation_right = True

            if self.rotation_right:
                self.actual_angle += self.step_angle
            else:
                self.actual_angle -= self.step_angle


            # Controller -> Update
            target_pos = self.target.position(self.surface_ref)
            self.controller.set_target_pos(target_pos)
            speed = self.speed()
            self.controller.speed = speed
            

            # Display -> Update
            self.frame.distances = self.distances
            self.frame.actual_angle = self.actual_angle
            self.frame.target_angle = self.controller.error_angle
            # Update dynamic texts
            batery = ceil((self.rover.resources.amount("ElectricCharge") / self.batery_limit) * 100)
            self.frame.texts[0][1]['text'] = f'{batery}%'
            self.frame.texts[0][1]['color'] = (0, 2.55*batery, 255 - 2.55*batery)

            target_distance = self.controller.target_dist
            distance_factor = target_distance / self.controller.safe_distance
            self.frame.texts[1][1]['text'] = f'{target_distance:.2f}m'
            self.frame.texts[1][1]['color'] = (0, 255*distance_factor, 255 - 255*distance_factor)

            limit_speed = self.controller.speed_target+10
            speed_factor = speed / limit_speed
            self.frame.texts[2][1]['text'] = f'{speed:.2f}m/s'
            self.frame.texts[2][1]['color'] = (0, 255 - 255*speed_factor, 255*speed_factor)

            error_angle = self.controller.error_angle
            angle_factor = abs(error_angle) / 20
            self.frame.texts[3][1]['text'] = f'{error_angle:.2f}Deg'
            self.frame.texts[3][1]['color'] = (0, 255 - 255*angle_factor, 255*angle_factor)


            # Map -> Update
            self.map.objects = [
                {
                    'pos': (self.longitude(), self.latitude()),
                    'radius': 5,
                    'color': (0, 0, 255)
                },
                {
                    'pos': (self.target_lng(), self.target_lat()),
                    'radius': 3,
                    'color': (255, 0, 0)
                }
            ]

    def set_angle_value(self, angle, value):
        self.distances[angle + self.mean_angle] = value

    def distances_to_positions(self):
        [print(angle) for (angle, i) in (range(self.distances))]