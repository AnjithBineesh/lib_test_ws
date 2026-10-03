import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from simdata.devices import SimLiDAR


class SimDataLiDARNode(Node):

    def __init__(self):
        super().__init__("simdata_lidar")

        self.lidar = SimLiDAR(
            dataset="deutsches_museum_2d",
            replay_speed=1.0
        )

        self.publisher = self.create_publisher(
            LaserScan,
            "/scan",
            10
        )

        self.get_logger().info(
            "SimData LiDAR ROS 2 node started."
        )

        self.timer = self.create_timer(
            0.001,
            self.publish_scan
        )

    def publish_scan(self):
        try:
            scan, replay_time = self.lidar.read()

            msg = LaserScan()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = scan.frame_id

            msg.angle_min = scan.angle_min
            msg.angle_max = scan.angle_max
            msg.angle_increment = scan.angle_increment

            msg.range_min = scan.range_min
            msg.range_max = scan.range_max

            msg.ranges = list(scan.ranges)

            self.publisher.publish(msg)

        except RuntimeError:
            self.get_logger().info(
                "LiDAR dataset finished."
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = SimDataLiDARNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        


if __name__ == "__main__":
    main()