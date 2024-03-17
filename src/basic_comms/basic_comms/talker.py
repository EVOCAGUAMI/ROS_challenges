import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class My_Publisher(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher = self.create_publisher(String, 'chit_chat', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized!')
        self.msg = String()

    def timer_callback(self):
        self.msg.data = 'Hola :)'
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
