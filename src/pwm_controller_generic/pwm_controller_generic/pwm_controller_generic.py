import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray


class PWMControllerGeneric(Node):
    def __init__(self):
        super().__init__('pwm_controller_generic')
        self.num_channels = 16
        self.min_pwm = 0.0
        self.max_pwm = 1.0
        self.pwm_values = [0.0] * self.num_channels

        self.pwm_pub = self.create_publisher(
            Float64MultiArray,
            'pwm',
            QoSProfile(depth=10)
        )

        self.pwm_sub = self.create_subscription(
            Float64MultiArray,
            'pwm_setpoints',
            self.pwm_setpoints_cb,
            QoSProfile(depth=10)
        )

    def pwm_setpoints_cb(self, msg):
        self.pwm_values = msg.data
        self.get_logger().info(f'Received PWM setpoints: {self.pwm_values}')
        self.apply_bounds_checking()

    def apply_bounds_checking(self):
        for i in range(self.num_channels):
            if self.pwm_values[i] < self.min_pwm:
                self.pwm_values[i] = self.min_pwm
                self.get_logger().warn(f'PWM channel {i} value capped to {self.min_pwm}')
            elif self.pwm_values[i] > self.max_pwm:
                self.pwm_values[i] = self.max_pwm
                self.get_logger().warn(f'PWM channel {i} value capped to {self.max_pwm}')

        self.publish_pwm()

    def publish_pwm(self):
        pwm_msg = Float64MultiArray()
        pwm_msg.data = self.pwm_values
        self.pwm_pub.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    pwm_controller = PWMControllerGeneric()
    rclpy.spin(pwm_controller)
    pwm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()