import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from enum import Enum

# Use the user's message type
from yolo_msgs.msg import DetectionArray as YoloDetectionArray

class STATE(Enum):
    IDLE = 0
    WANDERING = 1
    POSITIONING = 2
    SHOOTING = 3

class PingPong(Node):
    def __init__(self):
        super().__init__('ping_pong')

        # ---- Parameters (override with --ros-args -p key:=value) ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')
        self.declare_parameter('label', 'orange')
        self.declare_parameter('target_position', -1.75)
        self.declare_parameter('return_position', -1.48)
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('cooldown_sec', 4.0)          # prevents rapid retriggers while executing
        self.declare_parameter('min_interval_sec', 5.0)     # >= 5 s since last movement

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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
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

        self.curr_state = STATE.IDLE
        self.last_cmd = None

    def on_detections(self, msg: YoloDetectionArray):
        print(f"curr_state={self.curr_state.name}")
        if self._busy:
            return

        now = time.time()
        if (now - self._last_move_time) < self.min_interval:
            # Too soon since the last movement
            return
        
        if self.curr_state == STATE.IDLE:
            self.curr_state = STATE.WANDERING

        # yolo_msgs/Detection has fields like: class_name (string), score (float32), ...
        target_det = None

        if self.curr_state in [STATE.WANDERING, STATE.POSITIONING]:
            for det in msg.detections:
                if ((getattr(det, "class_name", "") == self.label) and 
                    (float(getattr(det, "score", 0.0)) >= self.min_score)):
                    target_det = det
                    break

        if target_det is not None:
            self.curr_state = STATE.POSITIONING
            self.aim(target_det)
        elif self.curr_state in [STATE.WANDERING, STATE.POSITIONING]:
            self.curr_state = STATE.WANDERING
            self.rotate()

        if self.curr_state == STATE.SHOOTING:
            self.get_logger().info("target detected — triggering motor movement.")
            self._last_move_time = now
            self.send_trajectory()
            self._set_busy()

    def aim(self, det):
        cx = 320
        kp = 0.001

        det_x = int(det.bbox.center.position.x)
        error = cx - det_x

        msg = Twist()
        if abs(error) > 10:
            msg.angular.z = kp * error
        else:
            msg.angular.z = 0.0
            self.curr_state = STATE.SHOOTING

        self.cmd_pub.publish(msg)
        self.last_cmd = msg.angular.z

    def rotate(self):
        rad = 0.1

        msg = Twist()
        if self.last_cmd is not None and self.last_cmd > 0.0:
            msg.angular.z = -rad
        else:
            msg.angular.z = rad

        self.cmd_pub.publish(msg)
        self.last_cmd = msg.angular.z
        
    def send_trajectory(self):
        # 3-point trajectory:
        # t=1s -> target; t=2s -> hold at target ~1s; t=3s -> back to 0
        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        pt1 = JointTrajectoryPoint()
        pt1.positions = [self.target_pos]
        pt1.time_from_start.sec = 0

        pt2 = JointTrajectoryPoint()
        pt2.positions = [self.target_pos]
        pt2.time_from_start.sec = 1

        pt3 = JointTrajectoryPoint()
        pt3.positions = [self.return_pos]
        pt3.time_from_start.sec = 3

        traj.points = [pt1, pt2, pt3]
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
        self.curr_state = STATE.IDLE


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
