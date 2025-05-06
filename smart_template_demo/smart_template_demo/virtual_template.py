import rclpy
import numpy as np
import time

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException, TransformException
from tf2_geometry_msgs import do_transform_pose
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
# Implements the robot publishers
#
# Publishes:   
# '/end_effector_pose'    (geometry_msgs.msg.PoseStamped)  - [m] robot frame
# '/joint_states'         (sensor_msgs.msg.JointState)     - [m] robot frame
#
# Subscribes:
# '/desired_position'     (geometry_msgs.msg.PoseStamped)  - [m] robot frame
# '/desired_command'      (std_msgs.msg.String)
# 
#########################################################################

class JointInfo:
    def __init__(self):
        self.names = []
        self.channels = []
        self.limits_lower = []
        self.limits_upper = []
        self.m_to_count = []
        self.count_to_m = []

    # Add a new joint to the list
    def add(self, name, channel, lower, upper, m_to_count, count_to_m):
        self.names.append(name)
        self.channels.append(channel)
        self.limits_lower.append(lower)
        self.limits_upper.append(upper)
        self.m_to_count.append(m_to_count)
        self.count_to_m.append(count_to_m)

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

#### Subscribed topics ###################################################

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription_desired_position = self.create_subscription(PoseStamped, '/desired_position', self.desired_position_callback, 10)
        self.subscription_desired_position # prevent unused variable warning

        self.subscription_desired_command = self.create_subscription(String, '/desired_command', self.desired_command_callback, 10)
        self.subscription_desired_command # prevent unused variable warning

#### Published topics ###################################################

        # Current position
        self.timer_ee = self.create_timer(0.3, self.timer_ee_pose_callback)
        self.publisher_ee_pose = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.timer_joints = self.create_timer(0.1, self.timer_joints_callback)
        self.publisher_joint_states = self.create_publisher(JointState, '/joint_states', 10)

#### Node initialization ###################################################

        # Initial home position - currently initializing in (0,0,0)
        self.current_joints = np.array([0.0, 0.0, 0.0])
        self.desired_joints = np.array([0.0, 0.0, 0.0])

        # Motion step for simulation
        self.joints_sim_step = np.array([0.0005, 0.001, 0.0005])

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
        raw_m_to_count = {}
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
                        lower = float(limit_elem.attrib.get('lower', 'nan'))
                        upper = float(limit_elem.attrib.get('upper', 'nan'))
                        raw_limits[name] = (lower, upper)
                    except ValueError:
                        self.get_logger().warn(f"Invalid limit values for joint '{name}'")
                # m_to_count
                m_elem = joint_elem.find('m_to_count')
                if m_elem is not None:
                    try:
                        m = float(m_elem.text.strip())
                        raw_m_to_count[name] = m
                    except ValueError:
                        self.get_logger().warn(f"Invalid m_to_count value for joint '{name}'")
        except ET.ParseError as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return None
        # Sort by channel order (A, B, C, ...)
        sorted_joints = sorted(raw_channels.items(), key=lambda item: item[1])
        for name, channel in sorted_joints:
            lower, upper = raw_limits.get(name, (float('-inf'), float('inf')))
            m_to_count = raw_m_to_count.get(name, 1.0)
            count_to_m = 1.0 / m_to_count if m_to_count != 0.0 else 0.0
            joint_info.add(name, channel, lower, upper, m_to_count, count_to_m)
            self.get_logger().info(f"{name}: channel={channel}, limits=({lower:.2f}, {upper:.2f}) m, "
                        f"m_to_count={m_to_count}, count_to_m={count_to_m}")
        return joint_info

    # Get current robot joint values [m]
    def get_joints(self) -> np.ndarray:
        return self.current_joints
        
    # Get current joints error [m]
    def get_joints_err(self) -> np.ndarray:
        err_joints = self.current_joints - self.desired_joints
        self.get_logger().info(f'Joint errors [m]: {err_joints}')
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
        self.get_logger().info(f'Error (joint/m): {err_joints}, FK error: {err_position}')
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
                    f"{self.joints.names[i]} value {value:.2f} m out of bounds "
                    f"[{lower:.2f}, {upper:.2f}] — clipping to limit."
                )
            capped = max(lower, min(value, upper))
            final_values.append(capped)
        return final_values

    # Sends a movement command to all joints based on the goal [x, y, z] in m.
    def position_control(self, goal: np.ndarray):
        self.desired_joints = self.ik_model(goal) 
        self.desired_joints = self.check_limits(self.desired_joints)      

    # Emulate one robot motion step
    def emulate_motion(self):
        delta = self.desired_joints - self.current_joints
        step = np.minimum(np.absolute(delta), self.joints_sim_step)
        self.current_joints = self.current_joints + np.multiply(np.sign(delta), step)

    # Loops robot emulation until convergence (in a non-blocking way)
    def start_emulated_motion_until_converged(self, eps=0.0001):
        def motion_loop():
            self.get_logger().debug("Starting background motion emulation loop")
            while True:
                self.emulate_motion()
                joints_err = self.get_joints_err()
                if all(abs(e) < eps for e in joints_err):
                    break
                time.sleep(0.1)
            self.get_logger().debug("Finished background motion emulation")
        thread = threading.Thread(target=motion_loop, daemon=True)
        thread.start()

#### Listening callbacks ###################################################

    # A request for desired position was sent
    def desired_position_callback(self, msg: PoseStamped):
        try:
            # Transform goal to base_link frame
            transform = self.tf_buffer.lookup_transform(target_frame='base_link', source_frame=msg.header.frame_id, time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            pose_in_base = do_transform_pose(msg.pose, transform)

            self.get_logger().info(f"Transform (world -> base_link): T = {transform.transform.translation}, R = {transform.transform.rotation}")
            self.get_logger().info(f"Input pose: {msg.pose.position}")
            self.get_logger().info(f"Transformed pose in base_link: {pose_in_base.position}")

            # Extract position in base_link
            goal_m = np.array([
                pose_in_base.position.x,
                pose_in_base.position.y,
                pose_in_base.position.z
            ])
            self.get_logger().info(f"Received {msg.header.frame_id}-frame goal, converted to base_link: x={goal_m[0]:.3f} m, y={goal_m[1]:.3f} m, z={goal_m[2]:.3f} m")
            # Compute desired joint values
            self.position_control(goal_m)
            self.start_emulated_motion_until_converged()
        except (LookupException, TimeoutException, TransformException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")

    # A request for desired command was sent
    def desired_command_callback(self, msg: String):
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

    # Publishes current robot end-effector pose
    def timer_ee_pose_callback(self):
        try:
            # Get transform from needle_link to world (i.e., where the needle is in world)
            t = self.tf_buffer.lookup_transform(target_frame='world', source_frame='needle_link', time=rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            pose_msg = PoseStamped()
            pose_msg.header = t.header
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            self.publisher_ee_pose.publish(pose_msg)
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

    # Publishes current robot joints
    def timer_joints_callback(self):
        # Read joint positions from robot encoders
        joints_m = self.get_joints()
        if joints_m is not None:
            # Update joint_state message to publish
            joint_state_msg = JointState()                
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joints.names
            joint_state_msg.position = [joints_m[0], joints_m[1], joints_m[2]] # In m (ROS2 tools expect meters)
            self.publisher_joint_states.publish(joint_state_msg)
            self.get_logger().debug('Joints [m]: %s'  % (joints_m))
            

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
