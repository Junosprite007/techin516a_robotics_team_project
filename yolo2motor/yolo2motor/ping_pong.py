import sys
import termios
import threading
import time
import random
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from enum import Enum

# Use the user's message type
from yolo_msgs.msg import DetectionArray as YoloDetectionArray

class ROBOT_STATE(Enum):
    IDLE = 0
    WANDERING = 1
    POSITIONING = 2
    SQUARING_BEFORE_APPROACH = 3
    APPROACHING = 4
    TURNING_TO_SHOOT = 5
    SHOOTING = 6
    SQUARING_BEFORE_RETURN = 7
    RETURNING = 8

class FLOW_STATE(Enum):
    INIT = 0
    SCANNING = 1
    AIMING = 2
    WAITING_PERMISSION = 3
    EXECUTING_SHOT = 4
    STANDBY = 5

class PingPong(Node):
    def __init__(self):
        super().__init__('ping_pong')

        self.settings = termios.tcgetattr(sys.stdin)

        # ---- Parameters (override with --ros-args -p key:=value) ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')
        # self.declare_parameter('label', 'orange')
        self.declare_parameter('target_position', 1.57)
        self.declare_parameter('target_position_mid', 1.67)
        self.declare_parameter('return_position', 0.0)
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('cooldown_sec', 4.0)          # prevents rapid retriggers while executing
        self.declare_parameter('min_interval_sec', 5.0)     # >= 5 s since last movement

        # ---- Load parameter values ----
        self.detections_topic = self.get_parameter('detections_topic').value
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        # self.label = self.get_parameter('label').value
        self.target_pos = float(self.get_parameter('target_position').value)
        self.target_pos_mid = float(self.get_parameter('target_position_mid').value)
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

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.front_distance = 0.0
        self.start_distance = 0.0
        self.approach_target_dist = 0.30 

        self.current_yaw = 0.0
        self.center_yaw = None
        self.shoot_yaw = 0.0

        self.use_lidar = False
        self.drive_time = 2.0
        self.drive_start_time = 0.0

        self.get_logger().info(
            f"\nusing yolo_msgs/DetectionArray on {self.detections_topic}; publishing to {self.controller_topic}"
        )

        # ---- Timing ----
        self._busy = False
        self._busy_timer = None
        self._last_move_time = 0.0  # epoch seconds

        self.label = None
        self.target_row_idx = 0
        self.robot_state = ROBOT_STATE.IDLE
        self.flow_state = FLOW_STATE.INIT
        self.last_cmd = None

        self.cli_thread = threading.Thread(target=self.run_cli, daemon=True)
        self.cli_thread.start()

    def angle_diff(self, target, current):
        diff = target - current
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff

    def run_cli(self):
        while rclpy.ok():
            if self.flow_state == FLOW_STATE.INIT:
                input("\n[SYSTEM] Square robot to the center of the board. Press ENTER to start.")

                if self.use_lidar:
                    while self.front_distance == 0.0:
                        self.get_logger().info("[SYSTEM] Waiting for valid lidar scan.")
                        time.sleep(1.0)
                    self.start_distance = self.front_distance  
                else:
                    self.start_distance = 0.6

                self.center_yaw = self.current_yaw
                self.start_distance = self.front_distance                
                self.flow_state = FLOW_STATE.SCANNING
            
            elif self.flow_state == FLOW_STATE.WAITING_PERMISSION:
                input(f"\n[SYSTEM] Target locked on '{self.label.upper()}'. Press ENTER when cleared.")
                
                aim_angle = self.angle_diff(self.current_yaw, self.center_yaw)
                
                target_y_offset = self.start_distance * math.tan(aim_angle)
                
                shoot_angle_relative = math.atan2(target_y_offset, self.approach_target_dist)
                self.shoot_yaw = self.center_yaw + shoot_angle_relative
                                
                self.robot_state = ROBOT_STATE.SQUARING_BEFORE_APPROACH
                self.flow_state = FLOW_STATE.EXECUTING_SHOT

            elif self.flow_state == FLOW_STATE.STANDBY:
                if not self._busy: 
                    input("\n[SYSTEM] Standing by. Press ENTER when ready.")
                    self.flow_state = FLOW_STATE.SCANNING
            
            time.sleep(0.1)

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        angle_inc = msg.angle_increment
        idx_15deg = int(0.26 / angle_inc) 
        front_ranges = np.concatenate((ranges[:idx_15deg], ranges[-idx_15deg:]))
        front_ranges = front_ranges[front_ranges > 0.05]
        if len(front_ranges) > 0:
            self.front_distance = np.min(front_ranges)

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        msg = Twist()
        
        if self.robot_state == ROBOT_STATE.SQUARING_BEFORE_APPROACH:
            diff = self.angle_diff(self.center_yaw, self.current_yaw)
            if abs(diff) > 0.05:
                msg.angular.z = float(0.8 * diff)
                self.cmd_pub.publish(msg)
            else:
                self.drive_start_time = time.time()
                self.robot_state = ROBOT_STATE.APPROACHING

        elif self.robot_state == ROBOT_STATE.APPROACHING:
            keep_driving = (self.front_distance > self.approach_target_dist) if self.use_lidar else (time.time() - self.drive_start_time < self.drive_time)

            if keep_driving:
                msg.linear.x = 0.15
                self.cmd_pub.publish(msg)
                self.get_logger().info("[SYSTEM] Moving forward...", throttle_duration_sec=0.5)
            else:
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)
                self.get_logger().info("[SYSTEM] Reached target point!")
                self.robot_state = ROBOT_STATE.TURNING_TO_SHOOT

        elif self.robot_state == ROBOT_STATE.TURNING_TO_SHOOT:
            diff = self.angle_diff(self.shoot_yaw, self.current_yaw)
            if abs(diff) > 0.05:
                msg.angular.z = float(0.8 * diff)
                self.cmd_pub.publish(msg)
            else:
                self.robot_state = ROBOT_STATE.SHOOTING

        elif self.robot_state == ROBOT_STATE.SHOOTING:
            if not self._busy:
                self._last_move_time = time.time()
                self.send_trajectory()
                self._set_busy()

        elif self.robot_state == ROBOT_STATE.SQUARING_BEFORE_RETURN:
            diff = self.angle_diff(self.center_yaw, self.current_yaw)
            if abs(diff) > 0.05:
                msg.angular.z = float(0.8 * diff)
                self.cmd_pub.publish(msg)
            else:
                self.drive_start_time = time.time()
                self.robot_state = ROBOT_STATE.RETURNING

        elif self.robot_state == ROBOT_STATE.RETURNING:
            keep_driving = (self.front_distance < self.start_distance) if self.use_lidar else (time.time() - self.drive_start_time < self.drive_time)

            if keep_driving:
                msg.linear.x = -0.15
                self.cmd_pub.publish(msg)
            else:
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)
                self.robot_state = ROBOT_STATE.IDLE
                self.flow_state = FLOW_STATE.STANDBY

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
            
            # dynamically cluster into rows based on y threshold
            y_threshold = 20.0
            rows = []
            current_row = []

            for d in grid_data:
                if not current_row:
                    current_row.append(d)
                else:
                    cy_avg = sum(item['cy'] for item in current_row) / len(current_row)
                    
                    if abs(d['cy'] - cy_avg) <= y_threshold:
                        current_row.append(d)
                    else:
                        rows.append(current_row)
                        current_row = [d]
            
            if current_row:
                rows.append(current_row)

            available_targets = []
            target_row_map = {}
                        
            for i, row in enumerate(rows):
                row.sort(key=lambda d: d['cx'])
                grid.extend(row)
                row_classes = [d['class'] for d in row]

                row_num = i + 1
                self.get_logger().info(f"row {row_num}: {row_classes}")
                available_targets.extend(row_classes)
                
                for cls in row_classes:
                    target_row_map[cls] = row_num

            if not available_targets:
                return
            
            self.label = random.choice(available_targets)
            self.target_row_idx = target_row_map[self.label]
            
            self.get_logger().debug(f"\nflow state: {self.flow_state}\nrobot state: {self.robot_state}\ntarget: {self.label}")
            self.get_logger().info(f"\n[SYSTEM] Target locked on {self.label.upper()} (Row {self.target_row_idx})")

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

    def aim(self, det):
        cx = 200 # center = 320
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
        shoot_pos = self.target_pos_mid if self.target_row_idx == 1 else self.target_pos

        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        pt1 = JointTrajectoryPoint()
        pt1.positions = [shoot_pos]
        pt1.time_from_start.sec = 0

        pt2 = JointTrajectoryPoint()
        pt2.positions = [shoot_pos]
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

        if self.flow_state == FLOW_STATE.EXECUTING_SHOT:
            self.robot_state = ROBOT_STATE.SQUARING_BEFORE_RETURN

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
