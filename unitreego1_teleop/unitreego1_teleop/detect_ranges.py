import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_ros.msg import SensorRanges


class Detect_ranges_node(Node):
    def __init__(self):
        super().__init__('detect_ranges')
        self.sensor_ranges_subscription = self.create_subscription(SensorRanges, '/sensor_ranges', self.sensor_ranges_callback, rclpy.qos.qos_profile_sensor_data)
        self.detectLeft = None
        self.detectRight = None
        self.detectFront = None

        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel",10)
        self.velLx = 0.0
        self.velLy = 0.0

        # self.timer_period = 0.1
        # self.timer = self.create_timer(self.timer_period, self.timer_callback_keyboard)
        self.get_logger().info('Detect ranges node started.')


    def sensor_ranges_callback(self, msg):
        if msg is not None:
            self.detectLeft = msg.left
            self.detectRight = msg.right
            self.detectFront = msg.front

            if self.detectLeft <= 0.15:
                self.velLx = 0.0
                self.velLy = -0.2
            elif self.detectRight <= 0.4:
                self.velLx = 0.0
                self.velLy = 0.2
            elif self.detectFront <= 0.14:
                self.velLx = -0.2
                self.velLy = 0.0
            else:
                self.velLx = 0.0
                self.velLy = 0.0

        # Crear el mensaje Twist y publicarlo
        twist_msg = Twist()
        twist_msg.linear.x = self.velLx
        twist_msg.linear.y = self.velLy
        # twist_msg.angular.z = self.velA
        self.cmd_vel_publisher.publish(twist_msg)
        


    # def timer_callback_keyboard(self):
        # if self.detectLeft <= 0.1:
        #     self.velLx = 0.0
        #     self.velLy = -0.2
        # elif self.detectRight <= 0.1:
        #     self.velLx = 0.0
        #     self.velLy = 0.2
        # elif self.detectFront <= 0.1:
        #     self.velLx = -0.2
        #     self.velLy = 0.0
        # else:
        #     self.velLx = 0.0
        #     self.velLy = 0.0

        # # Crear el mensaje Twist y publicarlo
        # twist_msg = Twist()
        # twist_msg.linear.x = self.velLx
        # twist_msg.linear.y = self.velLy
        # # twist_msg.angular.z = self.velA
        # self.cmd_vel_publisher.publish(twist_msg)

        

def main(args=None):
    rclpy.init(args=args)
    nodo = Detect_ranges_node()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()