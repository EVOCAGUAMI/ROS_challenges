import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class My_Subscriber(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.sub = self.create_subscription(String, 'chit_chat', self.listener_callback, 10)
        self.get_logger().info('Listener node initialized!')

    def listener_callback(self, msg):
        self.get_logger().info('Data: {}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    m_s = My_Subscriber()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()