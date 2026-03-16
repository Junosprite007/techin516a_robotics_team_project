import sys
import termios
import threading
import time
import random
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from enum import Enum

# Use the user's message type
from yolo_msgs.msg import DetectionArray as YoloDetectionArray

class ROBOT_STATE(Enum):
    IDLE = 0
    WANDERING = 1
    POSITIONING = 2
    SHOOTING = 3

class FLOW_STATE(Enum):
    INIT = 0
    SCANNING = 1
    AIMING = 2
    WAITING_PERMISSION = 3
    STANDBY = 4

class PingPong(Node):
    def __init__(self):
        super().__init__('ping_pong')

        self.settings = termios.tcgetattr(sys.stdin)

        # ---- Parameters (override with --ros-args -p key:=value) ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')
        # self.declare_parameter('label', 'orange')
        self.declare_parameter('target_position', -2.00)
        self.declare_parameter('return_position', -1.48)
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('cooldown_sec', 4.0)          # prevents rapid retriggers while executing
        self.declare_parameter('min_interval_sec', 5.0)     # >= 5 s since last movement

        # ---- Load parameter values ----
        self.detections_topic = self.get_parameter('detections_topic').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        # self.label = self.get_parameter('label').value
        self.target_pos = float(self.get_parameter('target_position').value)
        self.return_pos = float(self.get_parameter('return_position').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.cooldown = float(self.get_parameter('cooldown_sec').value)
        self.min_interval = float(self.get_parameter('min_interval_sec').value)

        self.valid_classes = ["car", "chair", "clock", "cup", "dog", "fire hydrant", "orange", "potted plant", "umbrella"]
        
        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            YoloDetectionArray, self.detections_topic, self.on_detections, 10
        )

        self.get_logger().info(
            f"\nusing yolo_msgs/DetectionArray on {self.detections_topic}; publishing to {self.controller_topic}"
        )

        # ---- Timing ----
        self._busy = False
        self._busy_timer = None
        self._last_move_time = 0.0  # epoch seconds

        self.label = None
        self.robot_state = ROBOT_STATE.IDLE
        self.flow_state = FLOW_STATE.INIT
        self.last_cmd = None

        self.cli_thread = threading.Thread(target=self.run_cli, daemon=True)
        self.cli_thread.start()

    def run_cli(self):
        while rclpy.ok():
            if self.flow_state == FLOW_STATE.INIT:
                input("\n[SYSTEM] Initialized. Press ENTER to start.")
                self.flow_state = FLOW_STATE.SCANNING
            
            elif self.flow_state == FLOW_STATE.WAITING_PERMISSION:
                input(f"\n[SYSTEM] Target locked on '{self.label.upper()}'. Press ENTER to fire.")
                self.robot_state = ROBOT_STATE.SHOOTING
                self.flow_state = FLOW_STATE.STANDBY

            elif self.flow_state == FLOW_STATE.STANDBY:
                if not self._busy: 
                    input("\n[SYSTEM] Standing by. Press ENTER when ready.")
                    self.flow_state = FLOW_STATE.SCANNING
            
            time.sleep(0.1)

    def on_detections(self, msg: YoloDetectionArray):
        if self._busy:
            return

        now = time.time()
        if (now - self._last_move_time) < self.min_interval:
            # Too soon since the last movement
            return
        
        # SCANNING TARGET
        if self.flow_state == FLOW_STATE.SCANNING:
            valid_detections = [
                d for d in msg.detections
                if float(getattr(d, "score", 0.0)) >= self.min_score
                and getattr(d, "class_name", "") in self.valid_classes
            ]

            if not valid_detections:
                return
            
            grid_data = [{'class': getattr(d, "class_name", ""),
                          'cx': d.bbox.center.position.x,
                          'cy': d.bbox.center.position.y} for d in valid_detections]
            
            grid_data.sort(key=lambda d: d['cy'])
            grid = []
            for i in range(0, len(grid_data), 3):
                row = grid_data[i:i+3]
                row.sort(key=lambda d: d['cx'])
                grid.extend(row)

            available_targets = [d['class'] for d in grid]
            self.get_logger().info(f"\ntargets: {available_targets}")
            if not available_targets:
                return
            
            self.label = random.choice(available_targets)
            self.get_logger().debug(f"\nflow state: {self.flow_state}\nrobot state: {self.robot_state}\ntarget: {self.label}")

            self.flow_state = FLOW_STATE.AIMING
            self.robot_state = ROBOT_STATE.IDLE
        
        # ADJUSTING POSE
        if self.flow_state == FLOW_STATE.AIMING:
            if self.robot_state == ROBOT_STATE.IDLE:
                self.robot_state = ROBOT_STATE.WANDERING

            # yolo_msgs/Detection has fields like: class_name (string), score (float32), ...
            target_det = None

            if self.robot_state in [ROBOT_STATE.WANDERING, ROBOT_STATE.POSITIONING]:
                for det in msg.detections:
                    if ((getattr(det, "class_name", "") == self.label) and 
                        (float(getattr(det, "score", 0.0)) >= self.min_score)):
                        target_det = det
                        break

            if target_det is not None:
                self.robot_state = ROBOT_STATE.POSITIONING
                self.aim(target_det)
            elif self.robot_state in [ROBOT_STATE.WANDERING, ROBOT_STATE.POSITIONING]:
                self.robot_state = ROBOT_STATE.WANDERING
                self.rotate()

        # SHOOTING
        if self.robot_state == ROBOT_STATE.SHOOTING:
            self.get_logger().debug(f"\nflow state: {self.flow_state}\nrobot state: {self.robot_state}\ntarget: {self.label}")
            self._last_move_time = now
            self.send_trajectory()
            self._set_busy()
            self.robot_state = ROBOT_STATE.IDLE

    def aim(self, det):
        cx = 320
        kp = 0.001

        det_x = int(det.bbox.center.position.x)
        error = cx - det_x

        msg = Twist()
        if abs(error) > 10:
            msg.angular.z = float(kp * error)
        else:
            msg.angular.z = 0.0
            if self.flow_state == FLOW_STATE.AIMING:
                self.flow_state = FLOW_STATE.WAITING_PERMISSION

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
        self.get_logger().debug("\ncooldown complete, ready for next trigger.")

def main():
    rclpy.init()
    node = PingPong()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
