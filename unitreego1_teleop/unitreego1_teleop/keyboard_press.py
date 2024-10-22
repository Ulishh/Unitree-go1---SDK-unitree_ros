import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from curtsies import Input


class Keyboard_press_node(Node):
    def __init__(self):
        super().__init__('keyboard_press')
        self.keyboard_publisher = self.create_publisher(String, "/keyboard_press",10)
        self.get_logger().info('Keyboard_press node started. Press WASD to move the robot')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback_keyboard)


    def timer_callback_keyboard(self):
        with Input(keynames='curtsies') as input_generator:
            for event in input_generator:
                msg = String()
                if event == 'w':  
                    msg.data = event
                elif event == 's':  
                    msg.data = event
                elif event == 'a':  
                    msg.data = event
                elif event == 'd':  
                    msg.data = event
                
                self.keyboard_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_node = Keyboard_press_node()
    rclpy.spin(keyboard_node)
    keyboard_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()