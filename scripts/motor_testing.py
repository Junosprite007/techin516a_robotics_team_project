import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class Catapult(Node):
    def __init__(self):
        super().__init__('catapult')

        # ---- Parameters (override with --ros-args -p key:=value) ----
        self.declare_parameter('controller_topic', '/gix_controller/joint_trajectory')
        self.declare_parameter('joint_name', 'gix')
        self.declare_parameter('max_vel', 6.0)  # 5.97 rad/s at 11.1V; 6.38 rad/s at 12V

        # ---- Load parameter values ----
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        self.max_vel = float(self.get_parameter('max_vel').value)

        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)

    def shoot(self, target_pos, target_vel):
        # cap velocity
        if target_vel > self.max_vel:
            self.get_logger().warn(f"vel {target_vel} exceeds max, capping at {self.max_vel}")
            target_vel = self.max_vel

        # calculate travel time
        travel_time = abs(target_pos / target_vel)
        
        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        p1 = JointTrajectoryPoint()
        p1.positions = [target_pos]
        p1.velocities = [target_vel]
        p1.time_from_start = Duration(sec=int(travel_time), nanosec=int((travel_time % 1) * 1e9))

        p2 = JointTrajectoryPoint()
        p2.positions = [target_pos + 0.1] 
        p2.velocities = [0.0]
        stop_time = travel_time + 0.1
        p2.time_from_start = Duration(sec=int(stop_time), nanosec=int((stop_time % 1) * 1e9))

        traj.points = [p1, p2]
        self.traj_pub.publish(traj)
        self.get_logger().info(f"fire! pos: {target_pos} rad, vel: {target_vel} rad/s")

def main():
    if len(sys.argv) < 3:
        print("usage: python3 motor_testing.py [pos] [vel]")
        return

    pos = float(sys.argv[1])
    vel = float(sys.argv[2])

    rclpy.init()
    node = Catapult()
    node.shoot(pos, vel)
    
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    