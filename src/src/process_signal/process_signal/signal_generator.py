import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np 
from scipy import signal 
from signal_msg.msg import SignalCustom

class Signal_Generator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        
        #1kHz
        self.signal_timer_period = 1e-3
        self.time = 0
        self.signal = 0

        #10Hz
        self.decomposed_timer_period = 1e-2
        self.dec_time = 0
        self.current_signal = 0

        self.declare_parameter('signal_type', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('default.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('default.frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('default.offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('default.phase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('default.time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('square.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('square.frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('square.offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('square.phase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('square.time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sawtooth.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sawtooth.frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sawtooth.offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sawtooth.phase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sawtooth.time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cosine.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cosine.frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cosine.offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cosine.phase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('cosine.time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tangent.amplitude', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tangent.frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tangent.offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tangent.phase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tangent.time', rclpy.Parameter.Type.DOUBLE)

        self.signal_msg = Float32()
        self.signal_dec_msg = SignalCustom()

        self.signal_pub = self.create_publisher(Float32, '/signal', 10)
        self.signal_params_pub = self.create_publisher(SignalCustom, '/signal_params', 10)
        
        self.signal_timer = self.create_timer(self.signal_timer_period, self.signal_timer_callback)
        self.decomposed_timer = self.create_timer(self.decomposed_timer_period, self.pub_timer_callback)

    def signal_timer_callback(self):

        self.time += self.signal_timer_period
        self.current_signal = self.get_parameter('signal_type').get_parameter_value().integer_value
        if self.current_signal == 0:
            self.f = self.get_parameter('default.frequency').get_parameter_value().double_value
            self.a = self.get_parameter('default.amplitude').get_parameter_value().double_value
            self.offset = self.get_parameter('default.offset').get_parameter_value().double_value
            self.phase = self.get_parameter('default.phase').get_parameter_value().double_value
            self.dec_time = self.get_parameter('default.time').get_parameter_value().double_value
            omega = 2 * np.pi * self .f
            wave = self.a * np.sin(omega * self.time + self.phase) + self.offset

        elif self.current_signal == 1:
            self.f = self.get_parameter('square.frequency').get_parameter_value().double_value
            self.a = self.get_parameter('square.amplitude').get_parameter_value().double_value
            self.offset = self.get_parameter('square.offset').get_parameter_value().double_value
            self.phase = self.get_parameter('square.phase').get_parameter_value().double_value
            self.dec_time = self.get_parameter('square.time').get_parameter_value().double_value
            omega = 2 * np.pi * self.f
            wave = self.a * signal.square(omega * self.time + self.phase) + self.offset

        elif self.current_signal == 2:
            self.f = self.get_parameter('sawtooth.frequency').get_parameter_value().double_value
            self.a = self.get_parameter('sawtooth.amplitude').get_parameter_value().double_value
            self.offset = self.get_parameter('sawtooth.offset').get_parameter_value().double_value
            self.phase = self.get_parameter('sawtooth.phase').get_parameter_value().double_value
            self.dec_time = self.get_parameter('sawtooth.time').get_parameter_value().double_value
            omega = 2 * np.pi * self.f
            wave = self.a * signal.sawtooth(omega * self.time + self.phase) + self.offset

        elif self.current_signal == 3:
            self.f = self.get_parameter('cosine.frequency').get_parameter_value().double_value
            self.a = self.get_parameter('cosine.amplitude').get_parameter_value().double_value
            self.offset = self.get_parameter('cosine.offset').get_parameter_value().double_value
            self.phase = self.get_parameter('cosine.phase').get_parameter_value().double_value
            self.dec_time = self.get_parameter('cosine.time').get_parameter_value().double_value
            omega = 2 * np.pi * self.f
            wave = self.a * np.cos(omega * self.time + self.phase) + self.offset
        
        elif self.current_signal == 4:
            self.f = self.get_parameter('tangent.frequency').get_parameter_value().double_value
            self.a = self.get_parameter('tangent.amplitude').get_parameter_value().double_value
            self.offset = self.get_parameter('tangent.offset').get_parameter_value().double_value
            self.phase = self.get_parameter('tangent.phase').get_parameter_value().double_value
            self.dec_time = self.get_parameter('tangent.time').get_parameter_value().double_value
            omega = 2 * np.pi * self.f
            wave = self.a * np.tan(omega * self.time + self.phase) + self.offset
        
        self.signal_msg.data = wave
        self.signal_pub.publish(self.signal_msg)

    def pub_timer_callback(self):
        self.signal_dec_msg.signaltype = self.current_signal
        self.signal_dec_msg.amplitude = self.a
        self.signal_dec_msg.frequency = self.f 
        self.signal_dec_msg.offset = self.offset
        self.signal_dec_msg.phase = self.phase
        self.signal_dec_msg.time = self.time
        self.signal_params_pub.publish(self.signal_dec_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Signal_Generator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()