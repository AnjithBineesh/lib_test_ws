import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from simdata.core.replay import ReplayClock
from simdata.devices import SimLiDAR, SimIMU, SimOdom


class SimDataReplayNode(Node):

    def __init__(self):
        super().__init__("simdata_replay")

        self.dataset = "deutsches_museum_2d"

        # =========================================================
        # Create one common dataset origin
        # =========================================================

        reference_odom = SimOdom(
            dataset=self.dataset,
            replay_speed=1.0,
        )

        self.origin_timestamp = (
            reference_odom.trajectory[0].timestamp
        )

        # =========================================================
        # Shared replay clock
        #
        # realtime=False because the ROS node itself controls
        # when samples are published.
        # =========================================================

        self.clock = ReplayClock(
            speed=1.0,
            realtime=False,
            origin_timestamp=self.origin_timestamp,
        )

        # =========================================================
        # Devices
        #
        # All devices use the same clock.
        # No device will sleep during read().
        # =========================================================

        self.lidar = SimLiDAR(
            dataset=self.dataset,
            realtime=False,
            clock=self.clock,
        )

        self.odom = SimOdom(
            dataset=self.dataset,
            clock=self.clock,
        )

        # IMU currently uses its own stream.
        # It is also non-blocking.
        self.imu = SimIMU(
            dataset=self.dataset,
            replay_speed=1.0,
        )

        # =========================================================
        # Publishers
        # =========================================================

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan",
            10,
        )

        self.imu_pub = self.create_publisher(
            Imu,
            "/imu",
            10,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        # =========================================================
        # TF
        # =========================================================

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(self)
        )

        self.publish_static_transforms()

        # =========================================================
        # Replay state
        # =========================================================

        self.replay_start_wall = time.monotonic()
        self.replay_epoch = time.time()

        self.lidar_finished = False
        self.odom_finished = False
        self.imu_finished = False

        self.next_lidar = None
        self.next_odom = None
        self.next_imu = None

        # Prime all streams.
        self.load_next_lidar()
        self.load_next_odom()
        self.load_next_imu()

        # =========================================================
        # Central scheduler
        #
        # 1 ms resolution gives us enough precision for the
        # ~197 Hz IMU stream.
        # =========================================================

        self.scheduler_timer = self.create_timer(
            0.001,
            self.scheduler_callback,
        )

        self.get_logger().info(
            "SimData centralized replay node started."
        )

        self.get_logger().info(
            "LiDAR: ~37 Hz | IMU: ~197 Hz | Odom: ~37 Hz"
        )

    # =============================================================
    # Replay time
    # =============================================================

    def current_replay_time(self):

        elapsed = (
            time.monotonic()
            - self.replay_start_wall
        )

        return elapsed * self.clock.speed

    # =============================================================
    # Convert replay time to ROS timestamp
    # =============================================================

    def replay_timestamp(self, replay_time):

        timestamp = (
            self.replay_epoch
            + replay_time
        )

        sec = int(timestamp)

        nanosec = int(
            (timestamp - sec)
            * 1_000_000_000
        )

        return sec, nanosec

    # =============================================================
    # Load next LiDAR sample
    # =============================================================

    def load_next_lidar(self):

        if self.lidar_finished:
            return

        try:

            self.next_lidar = self.lidar.read()

        except RuntimeError:

            self.next_lidar = None
            self.lidar_finished = True

            self.get_logger().info(
                "LiDAR replay finished."
            )

    # =============================================================
    # Load next Odom sample
    # =============================================================

    def load_next_odom(self):

        if self.odom_finished:
            return

        try:

            self.next_odom = self.odom.read()

        except RuntimeError:

            self.next_odom = None
            self.odom_finished = True

            self.get_logger().info(
                "Odometry replay finished."
            )

    # =============================================================
    # Load next IMU sample
    # =============================================================

    def load_next_imu(self):

        if self.imu_finished:
            return

        try:

            self.next_imu = self.imu.read()

        except RuntimeError:

            self.next_imu = None
            self.imu_finished = True

            self.get_logger().info(
                "IMU replay finished."
            )

    # =============================================================
    # Central scheduler
    # =============================================================

    def scheduler_callback(self):

        current_time = (
            self.current_replay_time()
        )

        # ---------------------------------------------------------
        # Odom
        # ---------------------------------------------------------

        while (
            self.next_odom is not None
            and self.next_odom[1] <= current_time
        ):

            odom, replay_time = (
                self.next_odom
            )

            self.publish_odom(
                odom,
                replay_time,
            )

            self.load_next_odom()

        # ---------------------------------------------------------
        # LiDAR
        # ---------------------------------------------------------

        while (
            self.next_lidar is not None
            and self.next_lidar[1] <= current_time
        ):

            scan, replay_time = (
                self.next_lidar
            )

            self.publish_lidar(
                scan,
                replay_time,
            )

            self.load_next_lidar()



        # ---------------------------------------------------------
        # IMU
        # ---------------------------------------------------------

        while (
            self.next_imu is not None
            and self.next_imu[1] <= current_time
        ):

            imu, replay_time = (
                self.next_imu
            )

            self.publish_imu(
                imu,
                replay_time,
            )

            self.load_next_imu()

        # ---------------------------------------------------------
        # End condition
        # ---------------------------------------------------------

        if (
            self.lidar_finished
            and self.odom_finished
            and self.imu_finished
        ):

            self.scheduler_timer.cancel()

            self.get_logger().info(
                "SimData replay completed."
            )

    # =============================================================
    # Static TF
    # =============================================================

    def publish_static_transforms(self):

        transforms = []

        # ---------------------------------------------------------
        # base_link -> imu_link
        # ---------------------------------------------------------

        imu_tf = TransformStamped()

        imu_tf.header.frame_id = "base_link"
        imu_tf.child_frame_id = "imu_link"

        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.0

        imu_tf.transform.rotation.x = 0.0
        imu_tf.transform.rotation.y = 0.0
        imu_tf.transform.rotation.z = 0.0
        imu_tf.transform.rotation.w = 1.0

        # ---------------------------------------------------------
        # base_link -> horizontal_laser_link
        # ---------------------------------------------------------

        lidar_tf = TransformStamped()

        lidar_tf.header.frame_id = "base_link"
        lidar_tf.child_frame_id = (
            "horizontal_laser_link"
        )

        lidar_tf.transform.translation.x = 0.0
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.0

        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0

        # ---------------------------------------------------------
        # base_footprint -> base_link
        # ---------------------------------------------------------

        footprint_tf = TransformStamped()
        footprint_tf.header.frame_id = "base_link"
        footprint_tf.child_frame_id = "base_footprint"
        footprint_tf.transform.rotation.w = 1.0

        # Set stamp to 0 so it's always valid
        imu_tf.header.stamp.sec = 0
        imu_tf.header.stamp.nanosec = 0
        lidar_tf.header.stamp.sec = 0
        lidar_tf.header.stamp.nanosec = 0
        footprint_tf.header.stamp.sec = 0
        footprint_tf.header.stamp.nanosec = 0

        transforms.append(imu_tf)
        transforms.append(lidar_tf)
        transforms.append(footprint_tf)

        self.static_tf_broadcaster.sendTransform(
            transforms
        )

    # =============================================================
    # Publish LiDAR
    # =============================================================

    def publish_lidar(
        self,
        scan,
        replay_time,
    ):

        sec, nanosec = (
            self.replay_timestamp(
                replay_time
            )
        )

        msg = LaserScan()

        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec

        msg.header.frame_id = (
            scan.frame_id
        )

        msg.angle_min = (
            scan.angle_min
        )

        msg.angle_max = (
            scan.angle_max
        )

        msg.angle_increment = (
            scan.angle_increment
        )

        msg.range_min = (
            scan.range_min
        )

        msg.range_max = (
            scan.range_max
        )

        msg.ranges = list(
            scan.ranges
        )

        self.scan_pub.publish(msg)

    # =============================================================
    # Publish IMU
    # =============================================================

    def publish_imu(
        self,
        imu,
        replay_time,
    ):

        sec, nanosec = (
            self.replay_timestamp(
                replay_time
            )
        )

        msg = Imu()

        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec

        msg.header.frame_id = (
            imu.frame_id
        )

        msg.orientation.x = (
            imu.orientation_x
        )

        msg.orientation.y = (
            imu.orientation_y
        )

        msg.orientation.z = (
            imu.orientation_z
        )

        msg.orientation.w = (
            imu.orientation_w
        )

        msg.angular_velocity.x = (
            imu.angular_velocity_x
        )

        msg.angular_velocity.y = (
            imu.angular_velocity_y
        )

        msg.angular_velocity.z = (
            imu.angular_velocity_z
        )

        msg.linear_acceleration.x = (
            imu.linear_acceleration_x
        )

        msg.linear_acceleration.y = (
            imu.linear_acceleration_y
        )

        msg.linear_acceleration.z = (
            imu.linear_acceleration_z
        )

        self.imu_pub.publish(msg)

    # =============================================================
    # Publish Odom
    # =============================================================

    def publish_odom(
        self,
        odom,
        replay_time,
    ):

        sec, nanosec = (
            self.replay_timestamp(
                replay_time
            )
        )

        msg = Odometry()

        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec

        msg.header.frame_id = (
            odom.frame_id
        )

        msg.child_frame_id = (
            odom.child_frame_id
        )

        msg.pose.pose.position.x = (
            odom.position_x
        )

        msg.pose.pose.position.y = (
            odom.position_y
        )

        msg.pose.pose.position.z = (
            odom.position_z
        )

        msg.pose.pose.orientation.x = (
            odom.orientation_x
        )

        msg.pose.pose.orientation.y = (
            odom.orientation_y
        )

        msg.pose.pose.orientation.z = (
            odom.orientation_z
        )

        msg.pose.pose.orientation.w = (
            odom.orientation_w
        )

        msg.twist.twist.linear.x = (
            odom.linear_velocity_x
        )

        msg.twist.twist.linear.y = (
            odom.linear_velocity_y
        )

        msg.twist.twist.linear.z = (
            odom.linear_velocity_z
        )

        msg.twist.twist.angular.x = (
            odom.angular_velocity_x
        )

        msg.twist.twist.angular.y = (
            odom.angular_velocity_y
        )

        msg.twist.twist.angular.z = (
            odom.angular_velocity_z
        )

        self.odom_pub.publish(msg)

        # ---------------------------------------------------------
        # odom -> base_link
        # ---------------------------------------------------------

        tf = TransformStamped()

        tf.header.stamp.sec = sec
        tf.header.stamp.nanosec = nanosec

        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = (
            odom.position_x
        )

        tf.transform.translation.y = (
            odom.position_y
        )

        tf.transform.translation.z = (
            odom.position_z
        )

        tf.transform.rotation.x = (
            odom.orientation_x
        )

        tf.transform.rotation.y = (
            odom.orientation_y
        )

        tf.transform.rotation.z = (
            odom.orientation_z
        )

        tf.transform.rotation.w = (
            odom.orientation_w
        )

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):

    rclpy.init(args=args)

    node = SimDataReplayNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()