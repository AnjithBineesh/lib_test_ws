import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from simdata.devices import SimIMU


class SimDataIMUNode(Node):

    def __init__(self):
        super().__init__("simdata_imu")

        self.imu = SimIMU(
            dataset="deutsches_museum_2d",
            replay_speed=1.0,
        )

        self.publisher = self.create_publisher(
            Imu,
            "/imu",
            10,
        )

        self.get_logger().info(
            "SimData IMU ROS 2 node started."
        )

        self.timer = self.create_timer(
            0.001,
            self.publish_imu,
        )

    def publish_imu(self):
        try:
            sample, replay_time = self.imu.read()

            msg = Imu()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = sample.frame_id

            msg.orientation.x = sample.orientation_x
            msg.orientation.y = sample.orientation_y
            msg.orientation.z = sample.orientation_z
            msg.orientation.w = sample.orientation_w

            msg.angular_velocity.x = sample.angular_velocity_x
            msg.angular_velocity.y = sample.angular_velocity_y
            msg.angular_velocity.z = sample.angular_velocity_z

            msg.linear_acceleration.x = sample.linear_acceleration_x
            msg.linear_acceleration.y = sample.linear_acceleration_y
            msg.linear_acceleration.z = sample.linear_acceleration_z

            self.publisher.publish(msg)

        except RuntimeError:
            self.get_logger().info(
                "IMU dataset finished."
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = SimDataIMUNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
