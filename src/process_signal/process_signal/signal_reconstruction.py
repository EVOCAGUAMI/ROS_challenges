import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from math import sin, pi, cos

class SignalProcessor(Node):
    def __init__init__(self):
        super().__init__('signal_processor')
        self.subscription_signal = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.subscription_time = self.create_subscription(Float32, '/time', self.time_callback, 10)
        self.publisher_proc_signal = self.create_publisher(Float32, '/proc_signal', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s period (10 Hz)
        self.alpha = 0.5  # Hardcoded offset parameter
        self.phase_shift = 0.1  # Hardcoded phase shift parameter
        self.time = 0.0

    def signal_callback(self, msg):
        processed_signal = self.process_signal(msg.data)
        self.publisher_proc_signal.publish(processed_signal)
        self.get_logger().info(f'Processed Signal: {processed_signal.data}')

    def time_callback(self, msg):
        self.time = msg.data

    def timer_callback(self):
        pass  # You can perform additional tasks in the timer callback if needed

    def process_signal(self, original_signal):
        # Offset the signal
        offset_signal = original_signal + self.alpha

        # Reduce amplitude by half
        amplitude_scaled_signal = 0.5 * offset_signal

        # Add a phase shift
        phase_shifted_signal = amplitude_scaled_signal * sin(self.time + self.phase_shift)

        return Float32(data=phase_shifted_signal)

def main(args=None):
    rclpy.init(args=args)
    signal_processor = SignalProcessor()
    rclpy.spin(signal_processor)
    signal_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
