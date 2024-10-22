import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_ros.msg import SensorRanges


class Detect_ranges_node(Node):
    def __init__(self):
        super().__init__('detect_ranges')
        # Suscripcion para obtener los valores detectados de los sensores ultrasonicos
        self.sensor_ranges_subscription = self.create_subscription(SensorRanges, '/sensor_ranges', self.sensor_ranges_callback, rclpy.qos.qos_profile_sensor_data)
        # Variables para almacenar los valores de los sensores
        self.detectLeft = None
        self.detectRight = None
        self.detectFront = None
        # Publicador para la velocidad 
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel",10)
        self.velLx = 0.0
        self.velLy = 0.0

        self.get_logger().info('Detect ranges node started.')

    # Funcion para almacenar los datos de los sensores ultrasonicos y procesar los mismos
    def sensor_ranges_callback(self, msg):
        if msg is not None:
            self.detectLeft = msg.left
            self.detectRight = msg.right
            self.detectFront = msg.front
            # Condiciones para mover al robot en direccion contraria de donde detecte un objeto
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

        # Creacion del mensaje Twist para publicarlo
        twist_msg = Twist()
        twist_msg.linear.x = self.velLx
        twist_msg.linear.y = self.velLy
        self.cmd_vel_publisher.publish(twist_msg)

        

def main(args=None):
    rclpy.init(args=args)
    nodo = Detect_ranges_node()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()