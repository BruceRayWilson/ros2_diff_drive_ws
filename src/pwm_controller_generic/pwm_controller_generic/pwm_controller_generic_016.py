import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64, Float64MultiArray


class PwmControllerGeneric(Node):
    def __init__(self):
        super().__init__('pwm_controller_generic')
        self.get_logger().info('PWM Controller Node Started')

        # Create publisher for PWM values
        self.pwm_pub = self.create_publisher(Float64MultiArray, 'pwm_values', QoSProfile(depth=10))

        # Create subscriber for PWM commands
        self.pwm_sub = self.create_subscription(Float64MultiArray, 'pwm_commands', self.pwm_callback, QoSProfile(depth=10))

    def pwm_callback(self, msg):
        # Process incoming PWM commands
        pwm_values = msg.data
        self.get_logger().info(f'Received PWM commands: {pwm_values}')

        # Simulate PWM output (replace with actual hardware control)
        self.get_logger().info('Simulating PWM output...')

        # Publish PWM values
        pwm_msg = Float64MultiArray()
        pwm_msg.data = pwm_values
        self.pwm_pub.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    pwm_controller = PwmControllerGeneric()
    try:
        rclpy.spin(pwm_controller)
    except KeyboardInterrupt:
        pwm_controller.get_logger().info('KeyboardInterrupt caught')
    finally:
        pwm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
