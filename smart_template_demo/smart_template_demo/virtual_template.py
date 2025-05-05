import rclpy
import numpy as np
import time

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import xml.etree.ElementTree as ET
import threading

#########################################################################
#
# Virtual Template
#
# Description:
# This node implements a virtual node to emulate the SmartTemplate
# Implements the robot service and action servers
#
# Publishes:   
# '/stage/state/guide_pose'     (geometry_msgs.msg.PoseStamped)  - [mm] robot frame
# '/joint_states'               (sensor_msgs.msg.JointState)      - [m] robot frame
#
# Subscribe:
# '/desired_position'           (geometry_msgs.msg.PoseStamped)  - [mm] robot frame
# '/desired_command'            (std_msgs.msg.String)
# 
#########################################################################

class JointInfo:
    def __init__(self):
        self.names = []
        self.channels = []
        self.limits_lower = []
        self.limits_upper = []
        self.mm_to_count = []
        self.count_to_mm = []

    # Add a new joint to the list
    def add(self, name, channel, lower, upper, mm_to_count, count_to_mm):
        self.names.append(name)
        self.channels.append(channel)
        self.limits_lower.append(lower)
        self.limits_upper.append(upper)
        self.mm_to_count.append(mm_to_count)
        self.count_to_mm.append(count_to_mm)

    def index(self, joint_name: str) -> int:
        return self.names.index(joint_name)
    
class VirtualSmartTemplate(Node):

    def __init__(self):
        super().__init__('virtual_smart_template')      
        np.set_printoptions(precision=4, suppress=False, floatmode='maxprec_equal', linewidth=120)

    #### Get joint limits from robot description ###################################################
        self.declare_parameter('robot_description', '')
        urdf_str = self.get_parameter('robot_description').get_parameter_value().string_value
        self.joints = self.parse_joint_info_from_urdf(urdf_str)
        if self.joints is None:
            self.get_logger().fatal("Joint configuration could not be loaded from URDF.")
            return 

#### Published topics ###################################################

        # Current position
        timer_period_stage = 0.3  # seconds
        self.timer_stage = self.create_timer(timer_period_stage, self.timer_stage_pose_callback)
        self.publisher_stage_pose = self.create_publisher(PoseStamped, '/stage/state/guide_pose', 10)
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

#### Subscribed topics ###################################################
        self.subscription_desired_position = self.create_subscription(PoseStamped, '/desired_position', self.desired_position_callback, 10)
        self.subscription_desired_position # prevent unused variable warning

        self.subscription_desired_command = self.create_subscription(String, '/desired_command', self.desired_command_callback, 10)
        self.subscription_desired_command # prevent unused variable warning

#### Node initialization ###################################################

        # Initial home position - currently initializing in (0,0,0)
        self.current_joints = np.array([0.0, 0.0, 0.0])
        self.desired_joints = np.array([0.0, 0.0, 0.0])

        # Motion step for simulation
        self.joints_sim_step = np.array([0.1, 0.5, 0.1])

        # Flag to abort command
        self.abort = False   

#### Kinematic model ###################################################

    # Forward kinematics
    def fk_model(self, joints: np.ndarray) -> np.ndarray:
        x = joints[0]    # horizontal_joint  (channel A)
        y = joints[1]    # insertion_joint   (channel B) 
        z = joints[2]    # vertical_joint    (channel C)
        return np.array([x, y, z], dtype=float)

    # Inverse kinematics
    def ik_model(self, position: np.ndarray) -> np.ndarray:
        horizontal_joint = position[0]  # horizontal_joint  (channel A)
        insertion_joint = position[1]   # insertion_joint   (channel B) 
        vertical_joint = position [2]   # vertical_joint    (channel C)
        return np.array([horizontal_joint, insertion_joint, vertical_joint], dtype=float)
    
#### Internal functions ###################################################

    # Load joint information from URDF
    def parse_joint_info_from_urdf(self, urdf_str: str) -> JointInfo:
        raw_channels = {}
        raw_limits = {}
        raw_mm_to_count = {}
        joint_info = JointInfo()
        try:
            root = ET.fromstring(urdf_str)
            for joint_elem in root.findall('joint'):
                name = joint_elem.attrib.get('name')
                if not name:
                    continue
                # Channel
                channel_elem = joint_elem.find('channel')
                if channel_elem is not None:
                    raw_channels[name] = channel_elem.text.strip()
                # Limits
                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    try:
                        lower = float(limit_elem.attrib.get('lower', 'nan')) * 1000
                        upper = float(limit_elem.attrib.get('upper', 'nan')) * 1000
                        raw_limits[name] = (lower, upper)
                    except ValueError:
                        self.get_logger().warn(f"Invalid limit values for joint '{name}'")
                # mm_to_count
                mm_elem = joint_elem.find('mm_to_count')
                if mm_elem is not None:
                    try:
                        mm = float(mm_elem.text.strip())
                        raw_mm_to_count[name] = mm
                    except ValueError:
                        self.get_logger().warn(f"Invalid mm_to_count value for joint '{name}'")
        except ET.ParseError as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return None
        # Sort by channel order (A, B, C, ...)
        sorted_joints = sorted(raw_channels.items(), key=lambda item: item[1])
        for name, channel in sorted_joints:
            lower, upper = raw_limits.get(name, (float('-inf'), float('inf')))
            mm_to_count = raw_mm_to_count.get(name, 1.0)
            count_to_mm = 1.0 / mm_to_count if mm_to_count != 0.0 else 0.0
            joint_info.add(name, channel, lower, upper, mm_to_count, count_to_mm)
            self.get_logger().info(f"{name}: channel={channel}, limits=({lower:.2f}, {upper:.2f}) mm, "
                        f"mm_to_count={mm_to_count}, count_to_mm={count_to_mm}")
        return joint_info

    # Get current robot joint values [mm]
    def get_joints(self) -> np.ndarray:
        return self.current_joints
        
    # Get current joints error [mm]
    def get_joints_err(self) -> np.ndarray:
        err_joints = self.current_joints - self.desired_joints
        self.get_logger().info(f'Joint errors [mm]: {err_joints}')
        return (err_joints)

    # Get current robot position
    def get_position(self) -> np.ndarray:
        joints = self.get_joints()
        if joints is None:
            return None
        return self.fk_model(joints)
                        
    # Get current robot position error
    def get_position_error(self) -> np.ndarray:
        err_joints = self.get_joints_err()
        if err_joints is None:
            return None
        err_position = self.fk_model(err_joints)
        self.get_logger().info(f'Error (joint/mm): {err_joints}, FK error: {err_position}')
        return err_position

    # Calculate euclidean error
    def error_3d(self, err: np.ndarray) -> float:
        return float(np.linalg.norm(err))
    
    # TODO: Check best implementation for this using Galil
    # Abort any ongoing motion (stop where it is)
    def abort_motion(self):
        self.abort = True
        self.galil.GCommand('SH')
        self.get_logger().info('ABORT')

    # Checks each joint value to be withing the joint's defined limits
    # The order of joint_values must match self.joints.names
    def check_limits(self, joint_values: list[float]) -> list[float]:
        if len(joint_values) != len(self.joints.names):
            raise ValueError("Length of joint_values does not match number of joints")
        final_values = []
        for i, value in enumerate(joint_values):
            lower = self.joints.limits_lower[i]
            upper = self.joints.limits_upper[i]
            if value < lower or value > upper:
                self.get_logger().warn(
                    f"{self.joints.names[i]} value {value:.2f} mm out of bounds "
                    f"[{lower:.2f}, {upper:.2f}] — clipping to limit."
                )
            capped = max(lower, min(value, upper))
            final_values.append(capped)
        return final_values

    # Sends a movement command to all joints based on the goal [x, y, z] in mm.
    def position_control(self, goal: np.ndarray):
        self.desired_joints = self.ik_model(goal) 
        self.desired_joints = self.check_limits(self.desired_joints)      

    # Emulate one robot motion step
    def emulate_motion(self):
        delta = self.desired_joints - self.current_joints
        step = np.minimum(np.absolute(delta), self.joints_sim_step)
        self.current_joints = self.current_joints + np.multiply(np.sign(delta), step)

    # Loops robot emulation until convergence (in a non-blocking way)
    def start_emulated_motion_until_converged(self, eps=0.01):
        def motion_loop():
            self.get_logger().debug("Starting background motion emulation loop")
            while True:
                self.emulate_motion()
                joints_err = self.get_joints_err()
                if all(abs(e) < eps for e in joints_err):
                    break
                time.sleep(0.05)
            self.get_logger().debug("Finished background motion emulation")
        thread = threading.Thread(target=motion_loop, daemon=True)
        thread.start()

#### Listening callbacks ###################################################

    # A request for desired position was sent
    def desired_position_callback(self, msg):
        goal = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.get_logger().info(f'Received request: x={goal[0]}, y={goal[1]}, z={goal[2]}')
        self.position_control(goal)
        self.start_emulated_motion_until_converged()

    # A request for desired command was sent
    def desired_command_callback(self, msg):
        command =  msg.data
        self.get_logger().debug('Received command request')
        self.get_logger().info('Command %s' %(command))
        if command == 'HOME':
            goal = np.array([0.0, 0.0, 0.0])
            self.position_control(goal)
            self.start_emulated_motion_until_converged()
        elif command == 'RETRACT':
            position = self.get_position()
            goal = np.array([position[0], 0.0, position[2]])
            self.position_control(goal)
            self.start_emulated_motion_until_converged()
        elif command == 'ABORT':
            self.abort_motion()

#### Publishing callbacks ###################################################

    # Publishes current robot pose
    def timer_stage_pose_callback(self):
        # Read joint positions from robot encoders
        joints_mm = self.get_joints()
        if joints_mm is not None:
            position = self.fk_model(joints_mm)
            # Construct robot message to publish             
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'needle_link'
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            self.publisher_stage_pose.publish(msg)
            self.get_logger().debug('smart_template [mm]: x=%f, y=%f, z=%f in %s frame'  % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.frame_id))
            # Update joint_state message to publish
            joint_state_msg = JointState()                
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joints.names
            joint_state_msg.position = [0.001*joints_mm[0], 0.001*joints_mm[1], 0.001*joints_mm[2]] # Convert from mm to m (ROS2 tools expect meters)
            self.publisher_joint_states.publish(joint_state_msg)

def main(args=None):

    rclpy.init(args=args)

    virtual_template = VirtualSmartTemplate()
    virtual_template.get_logger().info('VIRTUAL SmartTemplate ready')

    # rclpy.spin(virtual_template)

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(virtual_template, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    virtual_template.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
