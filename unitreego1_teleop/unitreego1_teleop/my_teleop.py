import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class My_teleop_node(Node):
    def __init__(self):
        super().__init__('my_teleop')
        # Subscripcion para obtener la tecla presionada
        self.keyboard_subscription = self.create_subscription(String, '/keyboard_press', self.keyboard_callback, rclpy.qos.qos_profile_sensor_data)
        self.keyboard_press = String
        # Publicador para mandar la velocidad por /cmd_vel
        self.teleop_publisher = self.create_publisher(Twist, "/cmd_vel",10)
        self.velLx = 0.0
        self.velLy = 0.0

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback_keyboard)
        self.get_logger().info('My teleop node started.')

    # Funcion para leer el topico de keyboard_press y almacenar el dato en una variable 
    def keyboard_callback(self, msg):
        if msg is not None:
            self.keyboard_press = msg.data
        

    # Funcion para procesar la tecla presionada y publicar la velocidad lineal en X o Y
    def timer_callback_keyboard(self):
        if self.keyboard_press == 'w': # Avanzar
            self.velLx = 0.2
            self.velLy = 0.0
        elif self.keyboard_press == 'a': # Izquierda
            self.velLx = 0.0
            self.velLy = 0.2
        elif self.keyboard_press == 's': # Retroceder
            self.velLx = -0.2
            self.velLy = 0.0
        elif self.keyboard_press == 'd': # Derecha
            self.velLx = 0.0
            self.velLy = -0.2
        else: # Detenerse
            self.velLx = 0.0
            self.velLy = 0.0

        # Creacion del mensaje Twist para publicarlo
        twist_msg = Twist()
        twist_msg.linear.x = self.velLx
        twist_msg.linear.y = self.velLy
        self.teleop_publisher.publish(twist_msg)

        

def main(args=None):
    rclpy.init(args=args)
    nodo = My_teleop_node()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()