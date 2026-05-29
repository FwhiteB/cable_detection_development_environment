import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagneticFieldNode(Node):
    def __init__(self):
        super().__init__('magnetic_field_node')

        self.state_subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.state_estimation_callback,
            10,
        )
        self.magnetic_field_publisher = self.create_publisher(
            MagneticField,
            '/magnetic_field',
            10,
        )

    def state_estimation_callback(self, odometry_msg):
        magnetic_field_msg = MagneticField()
        magnetic_field_msg.header.stamp = odometry_msg.header.stamp
        magnetic_field_msg.header.frame_id = odometry_msg.header.frame_id

        self.magnetic_field_publisher.publish(magnetic_field_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
