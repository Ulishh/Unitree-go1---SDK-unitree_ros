import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_ros.msg import SensorRanges
import sys
import termios
import tty


class Detect_ranges_node(Node):
    def __init__(self):
        super().__init__('detect_ranges')
        self.sensor_ranges_subscription = self.create_subscription(SensorRanges, '/sensor_ranges', self.sensor_ranges_callback, rclpy.qos.qos_profile_sensor_data)
        self.detectLeft = None
        self.detectRight = None
        self.detectFront = None
        self.left = False
        self.right = False
        self.front = False

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel",10)
        self.velLx = 0.0
        self.velLy = 0.0

        self.get_logger().info('Detect ranges node started.')


    def sensor_ranges_callback(self, msg):
        if msg is not None:
            self.detectLeft = msg.left
            self.detectRight = msg.right
            self.detectFront = msg.front
            self.left = True
            self.right = True
            self.front = True

            if self.detectLeft <= 0.15:
                self.left = False
                self.get_logger().info('Detect left.')
            elif self.detectRight <= 0.4:
                self.right = False
                self.get_logger().info('Detect right.')
            elif self.detectFront <= 0.14:
                self.front = False
                self.get_logger().info('Detect front.')
            
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                # if self.front or self.right or self.left:
                tty.setraw(fd)
                key = sys.stdin.read(1)
                if key == 'w' and self.front == True:  # Avanzar
                    self.velLx = 0.2
                    self.velLy = 0.0
                elif key == 's':  # Retroceder
                    self.velLx = -0.2
                    self.velLy = 0.0
                elif key == 'a' and self.left == True:  # Girar izquierda
                    self.velLx = 0.0
                    self.velLy = 0.2
                elif key == 'd' and self.right == True:  # Girar derecha
                    self.velLx = 0.0
                    self.velLy = -0.2
                else: # Detenerse
                    self.velLx = 0.0
                    self.velLy = 0.0
                # Crear el mensaje Twist y publicarlo
                twist_msg = Twist()
                twist_msg.linear.x = self.velLx
                twist_msg.linear.y = self.velLy
                self.cmd_vel_publisher.publish(twist_msg)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        

        

def main(args=None):
    rclpy.init(args=args)
    nodo = Detect_ranges_node()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()