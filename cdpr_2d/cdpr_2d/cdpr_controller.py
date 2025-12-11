import math
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class CDPRController(Node):

    # __init__() function
    # Initializes ROS2 node, parameters, publishers, subscribers, and timers
    def __init__(self):
        super().__init__('cdpr_controller')
        qos_profile = QoSProfile(depth=10)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('plane_width', 1.0),
                ('plane_height', 1.0),
                ('effector_width', 0.1),
                ('effector_height', 0.1),
                ('pulley_radius', 0.05),
                ('left_pulley_position', []),
                ('right_pulley_position', []),
                ('segment_duration', 3.0),
                ('timers.control_loop_period', 0.02),
                ('timers.effector_pub_period', 0.033),
                ('timers.cable_pub_period', 0.05),
                ('timers.pulley_pub_period', 0.05),
            ]
        )
        self.plane_width = self.get_parameter('plane_width').value
        self.plane_height = self.get_parameter('plane_height').value
        self.effector_width = self.get_parameter('effector_width').value
        self.effector_height = self.get_parameter('effector_height').value
        self.pulley_radius = self.get_parameter('pulley_radius').value
        left_p_pos = self.get_parameter('left_pulley_position').value
        right_p_pos = self.get_parameter('right_pulley_position').value
        self.default_left_pulley = (0.00, -0.125, 1.70)
        self.default_right_pulley = (1.00, -0.125, 1.70)
        self.effector_z = -0.125
        if left_p_pos and len(left_p_pos) == 3:
            self.left_pulley_pos_xyz = tuple(left_p_pos)
        else:
            self.left_pulley_pos_xyz = self.default_left_pulley
        if right_p_pos and len(right_p_pos) == 3:
            self.right_pulley_pos_xyz = tuple(right_p_pos)
        else:
            self.right_pulley_pos_xyz = self.default_right_pulley
        self.segment_duration = self.get_parameter('segment_duration').value
        ctrl_period = self.get_parameter('timers.control_loop_period').value
        eff_period = self.get_parameter('timers.effector_pub_period').value
        cab_period = self.get_parameter('timers.cable_pub_period').value
        pul_period = self.get_parameter('timers.pulley_pub_period').value
        self.points: List[Tuple[float, float]] = []
        self.points_number = 0
        self.mode = None
        self.marker_received = False
        self.current_x = self.plane_width / 2.0
        self.current_y = self.plane_height / 2.0
        self.initial_x = self.current_x
        self.initial_y = self.current_y
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.segment_index = 0
        self.animation_progress = 0.0
        self.moving = False
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        self.segment_target_params = []
        self.effector_coordinates_publisher = self.create_publisher(
            Path, '/effector_coordinates', qos_profile)
        self.cable_parameters_publisher = self.create_publisher(
            Float32MultiArray, '/cable_parameters', qos_profile)
        self.pulley_parameters_publisher = self.create_publisher(
            Float32MultiArray, '/pulley_parameters', qos_profile)
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.coordinates_subscriber = self.create_subscription(
            Path, '/cdpr', self.path_callback, qos_profile)
        self.control_timer = self.create_timer(ctrl_period, self.control_loop)
        self.effector_timer = self.create_timer(
            eff_period, self.publish_effector_parameters)
        self.cable_timer = self.create_timer(
            cab_period, self.publish_cable_parameters)
        self.pulley_timer = self.create_timer(
            pul_period, self.publish_pulley_parameters)
        self.get_logger().info('CONTROLLER ACTIVATED. WAITING FOR COORDINATES...')

    # calculate_cable_parameters() function
    # Computes cable lengths and angles for given end-effector coordinates
    def calculate_cable_parameters(self, x: float, y: float):
        left_p_x, left_p_y, left_p_z = self.left_pulley_pos_xyz
        right_p_x, right_p_y, right_p_z = self.right_pulley_pos_xyz
        left_connection_x = x - (self.effector_width / 2.0)
        left_connection_y = y + (self.effector_height / 2.0)
        right_connection_x = x + (self.effector_width / 2.0)
        right_connection_y = y + (self.effector_height / 2.0)
        dz_left = self.effector_z - left_p_z
        dz_right = self.effector_z - right_p_z
        left_len = math.sqrt((left_connection_x - left_p_x) ** 2 +
                             (left_connection_y - left_p_y) ** 2 + dz_left ** 2)
        right_len = math.sqrt((right_connection_x - right_p_x) ** 2 +
                              (right_connection_y - right_p_y) ** 2 + dz_right ** 2)
        left_ang_rad = math.atan2(
            left_connection_x - left_p_x,
            left_p_z - left_connection_y)
        right_ang_rad = math.atan2(
            right_p_x - right_connection_x,
            right_p_z - right_connection_y)
        left_ang = math.degrees(left_ang_rad)
        right_ang = math.degrees(right_ang_rad)
        return left_len, right_len, left_ang, right_ang

    # calculate_pulley_movement() function
    # Computes cable length changes and pulley rotation for movement
    def calculate_pulley_movement(self, start_x, start_y, end_x, end_y):
        l1_start, l2_start, _, _ = self.calculate_cable_parameters(
            start_x, start_y)
        l1_end, l2_end, _, _ = self.calculate_cable_parameters(end_x, end_y)
        delta_l1 = l1_end - l1_start
        delta_l2 = l2_end - l2_start
        pulley1_angle = delta_l1 / self.pulley_radius
        pulley2_angle = delta_l2 / self.pulley_radius
        return delta_l1, delta_l2, pulley1_angle, pulley2_angle

    # path_callback() function
    # Receives Path message and sets trajectory mode based on points
    def path_callback(self, msg: Path):
        self.points = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            self.points.append((x, y))
        self.points_number = len(self.points)
        if self.points_number == 0:
            self.get_logger().error('ERROR. PATH WITH NO POINTS')
            return
        if self.points_number == 1:
            self.mode = 'single'
            self.get_logger().info('=' * 80)
            self.get_logger().info('1 POINT RECEIVED')
            self.handle_single_point()
        elif self.points_number == 2:
            self.mode = 'two_point'
            self.initial_x, self.initial_y = self.points[0]
            self.target_x, self.target_y = self.points[1]
            self.current_x, self.current_y = self.initial_x, self.initial_y
            self.segment_index = 0
            self.moving = True
            self.animation_progress = 0.0
            self.objective_achieved = False
            self.shutdown_scheduled = False
            l1_init, l2_init, _, _ = self.calculate_cable_parameters(
                self.initial_x, self.initial_y)
            l1_end, l2_end, _, _ = self.calculate_cable_parameters(
                self.target_x, self.target_y)
            self.target_left_cable_change = l1_end - l1_init
            self.target_right_cable_change = l2_end - l2_init
            self.target_left_pulley_angle = self.target_left_cable_change / self.pulley_radius
            self.target_right_pulley_angle = self.target_right_cable_change / self.pulley_radius
            self.get_logger().info('2 POINTS RECEIVED')
        else:
            self.mode = 'multi'
            self.get_logger().info(
                f'{self.points_number} POINTS RECEIVED')
            self.start_multi_point_trajectory()
        self.marker_received = True

    # handle_single_point() function
    # Instantly moves effector to single target point and publishes data
    def handle_single_point(self):
        x, y = self.points[0]
        self.current_x, self.current_y = x, y
        path_msg = Path()
        ps = PoseStamped()
        ps.pose.position.x = self.current_x
        ps.pose.position.y = self.current_y
        ps.pose.position.z = self.effector_z
        path_msg.poses = [ps]
        self.effector_coordinates_publisher.publish(path_msg)
        L1, L2, Q1, Q2 = self.calculate_cable_parameters(
            self.current_x, self.current_y)
        delta_l1, delta_l2, p1_ang, p2_ang = self.calculate_pulley_movement(
            self.plane_width / 2.0, self.plane_height / 2.0, self.current_x, self.current_y)
        cable_msg = Float32MultiArray()
        cable_msg.data = [L1, L2, Q1, Q2]
        self.cable_parameters_publisher.publish(cable_msg)
        pulley_msg = Float32MultiArray()
        pulley_msg.data = [delta_l1, delta_l2, p1_ang, p2_ang]
        self.pulley_parameters_publisher.publish(pulley_msg)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [p1_ang, p2_ang]
        self.joint_state_publisher.publish(joint_state_msg)
        # self.get_logger().info('=' * 80)
        self.get_logger().info('END EFFECTOR POSITION (X, Y)')
        self.get_logger().info(
            f'- TARGET POSITION: ({
                self.current_x:.3f}, {
                self.current_y:.3f}) m')
        self.get_logger().info('LENGTHS OF EACH CABLE (L1, L2)')
        self.get_logger().info(f'- L1: {L1:.3f} m, L2: {L2:.3f} m')
        self.get_logger().info('ANGLES OF EACH CABLE (Q1, Q2)')
        self.get_logger().info(f'- Q1: {Q1:.2f} °, Q2: {Q2:.2f} °')
        self.get_logger().info('MOVEMENT OF EACH PULLEY (P1, P2)')
        self.get_logger().info(
            f'- EXTENDED / COLLAPSED CABLE:  P1: {delta_l1:+.3f} m,   P2: {delta_l2:+.3f} m')
        self.get_logger().info(f'- ROTATION ANGLES (RADIANS):   P1: {
              p1_ang:+.3f} rad, P2: {p2_ang:+.3f} rad')
        self.get_logger().info('=' * 80)

    # start_multi_point_trajectory() function
    # Prepares segment parameters for multi-point trajectory
    def start_multi_point_trajectory(self):
        self.points_number = len(self.points)
        self.segment_index = 0
        self.initial_x, self.initial_y = self.points[0]
        self.current_x, self.current_y = self.initial_x, self.initial_y
        self.target_x, self.target_y = self.points[1]
        self.moving = True
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        self.segment_target_params = []
        for i in range(self.points_number - 1):
            sx, sy = self.points[i]
            ex, ey = self.points[i + 1]
            l1, l2, _, _ = self.calculate_cable_parameters(ex, ey)
            delta_l1, delta_l2, p1_angle, p2_angle = self.calculate_pulley_movement(
                sx, sy, ex, ey)
            self.segment_target_params.append({
                'start': (sx, sy),
                'end': (ex, ey),
                'end_lengths': (l1, l2),
                'delta_l': (delta_l1, delta_l2),
                'pulley_angles': (p1_angle, p2_angle)
            })
        self.get_logger().info(f'CREATED PATH WITH {self.points_number} POINTS AND {
            len(self.segment_target_params)} SEGMENTS')

    # control_loop() function
    # Moves effector incrementally and updates pulley/joint states
    def control_loop(self):
        if self.objective_achieved and not self.shutdown_scheduled:
            self.shutdown_scheduled = True
            return
        if not self.moving:
            return
        try:
            ctrl_period = self.get_parameter(
                'timers.control_loop_period').value
        except Exception:
            ctrl_period = 0.02
        if self.segment_duration <= 0:
            increment = 1.0
        else:
            increment = float(ctrl_period) / float(self.segment_duration)
        self.animation_progress += increment
        if self.animation_progress >= 1.0:
            self.animation_progress = 1.0
            self.current_x = self.target_x
            self.current_y = self.target_y
            dl1, dl2, p1_ang, p2_ang = self.calculate_pulley_movement(
                self.initial_x, self.initial_y, self.current_x, self.current_y)
            self.left_pulley_position += p1_ang
            self.right_pulley_position += p2_ang
            self.publish_joint_states()
            if self.mode == 'multi' and self.segment_index < (
                    self.points_number - 2):
                self.segment_index += 1
                self.initial_x, self.initial_y = self.points[self.segment_index]
                self.target_x, self.target_y = self.points[self.segment_index + 1]
                self.animation_progress = 0.0
            else:
                self.objective_achieved = True
                self.moving = False
                self.show_final_summary()
            return
        new_x = self.initial_x + \
            (self.target_x - self.initial_x) * self.animation_progress
        new_y = self.initial_y + \
            (self.target_y - self.initial_y) * self.animation_progress
        dl1, dl2, p1_ang, p2_ang = self.calculate_pulley_movement(
            self.current_x, self.current_y, new_x, new_y)
        self.left_pulley_position += p1_ang
        self.right_pulley_position += p2_ang
        self.publish_joint_states()
        self.current_x = new_x
        self.current_y = new_y

    # publish_effector_parameters() function
    # Publishes effector Path and transform for visualization
    def publish_effector_parameters(self):
        path_msg = Path()
        pose_current = PoseStamped()
        pose_current.pose.position.x = self.current_x
        pose_current.pose.position.y = self.current_y
        pose_current.pose.position.z = self.effector_z
        pose_target = PoseStamped()
        pose_target.pose.position.x = self.target_x
        pose_target.pose.position.y = self.target_y
        pose_target.pose.position.z = self.effector_z
        path_msg.poses = [pose_current, pose_target]
        self.effector_coordinates_publisher.publish(path_msg)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'end_effector'
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.effector_z
        t.transform.translation.z = self.current_y + 0.7
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)
        if self.moving:
            progress_percent = self.animation_progress * 100.0
            if self.mode == 'multi':
                self.get_logger().info(
                    f'PROGRESS [{self.segment_index + 1}/{max(1, self.points_number - 1)}] '
                    f'{progress_percent:.1f}% POS=({self.current_x:.3f}, {self.current_y:.3f}) '
                    f'TGT=({self.target_x:.3f}, {self.target_y:.3f})'
                )
            else:
                self.get_logger().info(
                    f'PROGRESS {
                        progress_percent:.1f}% - POS=({
                        self.current_x:.3f}, {
                        self.current_y:.3f}) ' f'TGT=({
                        self.target_x:.3f}, {
                        self.target_y:.3f})')

    # publish_cable_parameters() function
    # Publishes current cable lengths and angles
    def publish_cable_parameters(self):
        L1, L2, Q1, Q2 = self.calculate_cable_parameters(
            self.current_x, self.current_y)
        msg = Float32MultiArray()
        msg.data = [L1, L2, Q1, Q2]
        self.cable_parameters_publisher.publish(msg)

    # publish_pulley_parameters() function
    # Publishes pulley movement and rotation data
    def publish_pulley_parameters(self):
        if self.moving:
            seg_start_x, seg_start_y = self.initial_x, self.initial_y
            dl1, dl2, pa1, pa2 = self.calculate_pulley_movement(
                seg_start_x, seg_start_y, self.current_x, self.current_y)
        else:
            dl1 = dl2 = pa1 = pa2 = 0.0
        msg = Float32MultiArray()
        msg.data = [dl1, dl2, pa1, pa2]
        self.pulley_parameters_publisher.publish(msg)

    # publish_joint_states() function
    # Publishes joint states for ROS2 joint visualization
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pulley_position, self.right_pulley_position]
        self.joint_state_publisher.publish(msg)

    # show_final_summary() function
    # Prints detailed summary of effector movement and cable data
    def show_final_summary(self):
        if self.mode == 'two_point':
            final_left_length, final_right_length, final_left_angle, final_right_angle = \
                self.calculate_cable_parameters(self.current_x, self.current_y)
            initial_left_length, initial_right_length, initial_left_angle, initial_right_angle = \
                self.calculate_cable_parameters(self.initial_x, self.initial_y)
            self.get_logger().info('END EFFECTOR POSITIONS (X, Y)')
            self.get_logger().info(
                f'- INITIAL POSITION:  ({self.initial_x:.3f}, {self.initial_y:.3f}) m')
            self.get_logger().info(
                f'- FINAL POSITION:    ({self.current_x:.3f}, {self.current_y:.3f}) m')
            self.get_logger().info(
                f'- TARGET POSITION:   ({self.target_x:.3f}, {self.target_y:.3f}) m')
            self.get_logger().info('LENGTHS OF EACH CABLE (L1, L2)')
            self.get_logger().info(
                f'- INITIAL LENGTHS:       L1: {
                    initial_left_length:.3f} m,  L2: {
                    initial_right_length:.3f} m')
            self.get_logger().info(
                f'- FINAL LENGTHS:         L1: {
                    final_left_length:.3f} m,  L2: {
                    final_right_length:.3f} m')
            self.get_logger().info(f'- DIFFERENCE IN LENGTHS: L1: {self.target_left_cable_change:+.3f} m, L2: {
                  self.target_right_cable_change:+.3f} m')
            self.get_logger().info('ANGLES OF EACH CABLE (Q1, Q2)')
            self.get_logger().info(
                f'- INITIAL ANGLES:  Q1: {
                    initial_left_angle:.2f} °, Q2: {
                    initial_right_angle:.2f} °')
            self.get_logger().info(
                f'- FINAL ANGLES:    Q1: {
                    final_left_angle:.2f} °, Q2: {
                    final_right_angle:.2f} °')
            self.get_logger().info('MOVEMENT OF EACH PULLEY (P1, P2)')
            self.get_logger().info(
                f'- EXTENDED / COLLAPSED CABLE BY P1: {self.target_left_cable_change:+.3f} m')
            self.get_logger().info(
                f'- EXTENDED / COLLAPSED CABLE BY P2: {self.target_right_cable_change:+.3f} m')
            self.get_logger().info(f'- P1 ROTATING ANGLE: {self.target_left_pulley_angle:+.3f} rad ({
                  math.degrees(self.target_left_pulley_angle):+.2f} °)')
            self.get_logger().info(f'- P2 ROTATING ANGLE: {self.target_right_pulley_angle:+.3f} rad ({
                  math.degrees(self.target_right_pulley_angle):+.2f} °)')
            self.get_logger().info('=' * 80)
        elif self.mode == 'multi':
            self.get_logger().info(f'NUMBER OF POINTS: {self.points_number}')
            self.get_logger().info('POINTS THE EFFECTOR HAS PASSED THROUGH')
            for i, (x, y) in enumerate(self.points):
                self.get_logger().info(f'- POINT {i + 1}: ({x:.3f}, {y:.3f}) m')
            for i, (x, y) in enumerate(self.points):
                L1, L2, q1, q2 = self.calculate_cable_parameters(x, y)
                self.get_logger().info(f'DATA POSITION NUMBER {i + 1}')
                self.get_logger().info(f'- L1 = {L1:.3f} m, L2 = {L2:.3f} m')
                self.get_logger().info(f'- Q1 = {q1:.2f} °, Q2 = {q2:.2f} °')
            for i in range(self.points_number - 1):
                sx, sy = self.points[i]
                ex, ey = self.points[i + 1]
                L1_i, L2_i, _, _ = self.calculate_cable_parameters(sx, sy)
                L1_f, L2_f, _, _ = self.calculate_cable_parameters(ex, ey)
                L1_mov = L1_f - L1_i
                L2_mov = L2_f - L2_i
                P1_mov_rad = L1_mov / self.pulley_radius
                P2_mov_rad = L2_mov / self.pulley_radius
                self.get_logger().info(f'MOVEMENT BETWEEN POSITIONS {i + 1} AND {i + 2}')
                self.get_logger().info(f'- DL1 = {L1_mov:+.3f} m, DL2 = {L2_mov:+.3f} m')
                self.get_logger().info(
                    f'- P1 = {P1_mov_rad:+.3f} rad ({math.degrees(P1_mov_rad):+.2f} °)')
                self.get_logger().info(
                    f'- P2 = {P2_mov_rad:+.3f} rad ({math.degrees(P2_mov_rad):+.2f} °)')
            self.get_logger().info('=' * 80)
        elif self.mode == 'single':
            pass

    # __del__() function
    # Safely shuts down ROS2 node
    def __del__(self):
        try:
            self.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


# main() function
# Initializes ROS2 and starts the CDPR controller node
def main(args=None):
    rclpy.init(args=args)
    controller = CDPRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\nCLOSING CONTROLLER...')
    except Exception as e:
        print(f'ERROR: {e}')
    finally:
        pass


if __name__ == '__main__':
    main()
