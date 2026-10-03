import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from simdata.devices import SimOdom


class SimDataOdomNode(Node):
    def __init__(self):
        super().__init__("simdata_odom")

        self.odom = SimOdom(
            dataset="deutsches_museum_2d",
            replay_speed=1.0,
        )

        self.publisher = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        self.get_logger().info(
            "SimData Odometry ROS 2 node started."
        )

        self.timer = self.create_timer(
            0.001,
            self.publish_odom,
        )

    def publish_odom(self):
        try:
            sample, replay_time = self.odom.read()

            msg = Odometry()

            # ROS 2 timestamp
            msg.header.stamp = self.get_clock().now().to_msg()

            # Coordinate frames
            msg.header.frame_id = sample.frame_id
            msg.child_frame_id = sample.child_frame_id

            # Position
            msg.pose.pose.position.x = sample.position_x
            msg.pose.pose.position.y = sample.position_y
            msg.pose.pose.position.z = sample.position_z

            # Orientation
            msg.pose.pose.orientation.x = sample.orientation_x
            msg.pose.pose.orientation.y = sample.orientation_y
            msg.pose.pose.orientation.z = sample.orientation_z
            msg.pose.pose.orientation.w = sample.orientation_w

            # Linear velocity
            msg.twist.twist.linear.x = sample.linear_velocity_x
            msg.twist.twist.linear.y = sample.linear_velocity_y
            msg.twist.twist.linear.z = sample.linear_velocity_z

            # Angular velocity
            msg.twist.twist.angular.x = sample.angular_velocity_x
            msg.twist.twist.angular.y = sample.angular_velocity_y
            msg.twist.twist.angular.z = sample.angular_velocity_z

            self.publisher.publish(msg)

        except RuntimeError:
            self.get_logger().info(
                "Odometry dataset finished."
            )
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = SimDataOdomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
