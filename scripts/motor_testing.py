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

        # ---- Load parameter values ----
        self.controller_topic = self.get_parameter('controller_topic').value
        self.joint_name = self.get_parameter('joint_name').value

        # ---- Pub/Sub ----
        self.traj_pub = self.create_publisher(JointTrajectory, self.controller_topic, 10)

    def shoot(self, target_pos, target_time):

        if target_pos == 0.0:
            target_time = 3.0
            
        traj = JointTrajectory()
        traj.joint_names = [self.joint_name]

        p1 = JointTrajectoryPoint()
        p1.positions = [target_pos]
        p1.time_from_start = Duration(sec=int(target_time), nanosec=int((target_time % 1) * 1e9))

        traj.points = [p1]
        self.traj_pub.publish(traj)
        self.get_logger().info(f"fire! pos: {target_pos} rad, time: {target_time} s")

def main():
    if len(sys.argv) < 3:
        print("usage: python3 motor_testing.py [pos] [time]")
        return

    pos = float(sys.argv[1])
    time = float(sys.argv[2])

    rclpy.init()
    node = Catapult()
    node.shoot(pos, time)
    
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    