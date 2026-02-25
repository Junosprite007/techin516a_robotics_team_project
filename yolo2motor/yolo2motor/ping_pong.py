import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Use the user's message type
from yolo_msgs.msg import DetectionArray as YoloDetectionArray

class PingPong(Node):
    def __init__(self):
        super().__init__('ping_pong')

        # ---- Parameters (override with --ros-args -p key:=value) ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')
        self.declare_parameter('label', 'dog')
        self.declare_parameter('target_position', 0.5)
        self.declare_parameter('return_position', 1.6)
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('cooldown_sec', 4.0)          # prevents rapid retriggers while executing
        self.declare_parameter('min_interval_sec', 60.0)     # >= 60 s since last movement

        # ---- Load parameter values ----
        self.detections_topic = self.get_parameter('detections_topic').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        self.label = self.get_parameter('label').value
        self.target_pos = float(self.get_parameter('target_position').value)
        self.return_pos = float(self.get_parameter('return_position').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.cooldown = float(self.get_parameter('cooldown_sec').value)
        self.min_interval = float(self.get_parameter('min_interval_sec').value)

        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.sub = self.create_subscription(
            YoloDetectionArray, self.detections_topic, self.on_detections, 10
        )

        self.get_logger().info(
            f"Using yolo_msgs/DetectionArray on {self.detections_topic}; publishing to {self.controller_topic}"
        )

        # ---- Timing ----
        self._busy = False
        self._busy_timer = None
        self._last_move_time = 0.0  # epoch seconds

    def on_detections(self, msg: YoloDetectionArray):
        if self._busy:
            return

        now = time.time()
        if (now - self._last_move_time) < self.min_interval:
            # Too soon since the last movement
            return

        # yolo_msgs/Detection has fields like: class_name (string), score (float32), ...
        seen = any(
            (getattr(det, "class_name", "") == self.label) and
            (float(getattr(det, "score", 0.0)) >= self.min_score)
            for det in msg.detections
        )

        if seen:
            self.get_logger().info("target detected — triggering motor movement.")
            self._last_move_time = now
            self.send_trajectory()
            self._set_busy()

    def send_trajectory(self):
        # 3-point trajectory:
        # t=1s -> target; t=2s -> hold at target ~1s; t=3s -> back to 0
        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        pt1 = JointTrajectoryPoint()
        pt1.positions = [self.target_pos]
        pt1.time_from_start.sec = 1

        pt2 = JointTrajectoryPoint()
        pt2.positions = [self.return_pos]
        pt2.time_from_start.sec = 3

        traj.points = [pt1, pt2]
        self.traj_pub.publish(traj)

    def _set_busy(self):
        self._busy = True
        if self._busy_timer is not None:
            self._busy_timer.cancel()
        self._busy_timer = self.create_timer(self.cooldown, self._clear_busy)

    def _clear_busy(self):
        self._busy = False
        if self._busy_timer:
            self._busy_timer.cancel()
            self._busy_timer = None
        self.get_logger().debug("Cooldown complete; ready for next trigger.")


def main():
    rclpy.init()
    node = PingPong()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
