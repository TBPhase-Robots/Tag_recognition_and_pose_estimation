from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import Int64

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
import marker_maker

class Robot():
    id: int
    marker_publisher: Publisher

class Server(Node):
    next_id = 0
    active_robot_ids = []
    active_robots = []

    ids_publisher: Publisher = None

    def __init__(self):
        super().__init__("server")

        self.ids_publisher = self.create_publisher(Int32, '/ids', 10)
        self.register_subscription = self.create_subscription(Int32, '/register', self.register_callback, 10)

    def setup_robot(self, robot: Robot):
        robot.marker_publisher = self.create_publisher(Int64, f'/robot{robot.id}/markers', 10)

        marker = marker_maker.marker_to_int(robot.id)
        sleep(3)

        marker_msg = Int64()
        marker_msg.data = marker
        print('publishing marker')
        robot.marker_publisher.publish(marker_msg)


    def register_callback(self, msg: Int32):
        if msg.data == -1:
            self.active_robot_ids.append(self.next_id)
            robot = Robot()
            robot.id = self.next_id
            self.active_robots.append(robot)

            print(f'Sending id: {self.next_id}')

            id_msg = Int32()
            id_msg.data = self.next_id
            self.ids_publisher.publish(id_msg)

            self.next_id += 1
        else:
            id_count = self.active_robot_ids.count(msg.data)
            if id_count > 1:
                print(f'Duplicate id: {msg.data}')
            else:
                robot_index = self.active_robot_ids.index(msg.data)
                robot = self.active_robots[robot_index]
                self.setup_robot(robot)


def main():
    rclpy.init()

    server = Server()

    rclpy.spin(server)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
