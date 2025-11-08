import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

# COMPILING, LINKING, AND EXECUTING

# 1 POINT (FIXED POSITION)
# TERMINAL 1: colcon build --symlink-install
# TERMINAL 2: ros2 run cdpr_2d cdpr_controller
# TERMINAL 3: ros2 topic echo /effector_coordinates
# TERMINAL 4: ros2 topic echo /cable_parameters
# TERMINAL 5: ros2 topic echo /pulley_parameters
# TERMINAL 6:
# ros2 topic pub --once /cdpr nav_msgs/msg/Path \
# "{header: {frame_id: 'world'}, \
#   poses: [ \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.3, y: 0.3, z: 0.0}}} \
#   ] \
# }"

# 2 POINTS (INITIAL AND FINAL POSITION)
# TERMINAL 1: colcon build --symlink-install
# TERMINAL 2: ros2 run cdpr_2d cdpr_controller
# TERMINAL 3: ros2 topic echo /effector_coordinates
# TERMINAL 4: ros2 topic echo /cable_parameters
# TERMINAL 5: ros2 topic echo /pulley_parameters
# TERMINAL 6:
# ros2 topic pub --once /cdpr nav_msgs/msg/Path \
# "{header: {frame_id: 'world'}, \
#   poses: [ \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.3, y: 0.3, z: 0.0}}}, \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.7, y: 0.7, z: 0.0}}} \
#   ] \
# }"

# 3 OR MORE POINTS (TRAJECTORY)
# TERMINAL 1: colcon build --symlink-install
# TERMINAL 2: ros2 run cdpr_2d cdpr_controller
# TERMINAL 3: ros2 topic echo /effector_coordinates
# TERMINAL 4: ros2 topic echo /cable_parameters
# TERMINAL 5: ros2 topic echo /pulley_parameters
# TERMINAL 6:
# ros2 topic pub --once /cdpr nav_msgs/msg/Path \
# "{header: {frame_id: 'world'}, \
#   poses: [ \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.2, y: 0.2, z: 0.0}}}, \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.2, y: 0.8, z: 0.0}}}, \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.8, y: 0.8, z: 0.0}}}, \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.8, y: 0.2, z: 0.0}}}, \
#     {header: {frame_id: 'world'}, \
#      pose: {position: {x: 0.2, y: 0.2, z: 0.0}}} \
#   ] \
# }"


class CDPRController(Node):

    def __init__(self):
        super().__init__('cdpr_controller')
        self.plane_width = 1.0
        self.plane_height = 1.0
        self.effector_width = 0.1
        self.effector_height = 0.1
        self.pulley_radius = 0.05
        self.points = []
        self.marker_received = False
        self.mode = None
        self.points_number = 0
        self.current_x = 0.5
        self.current_y = 0.5
        self.initial_x = 0.5
        self.initial_y = 0.5
        self.target_x = 0.5
        self.target_y = 0.5
        self.segment_index = 0
        self.animation_progress = 0.0
        self.moving = False
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        self.segment_target_params = []
        qos_profile = QoSProfile(depth=10)
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
        self.control_timer = self.create_timer(0.5, self.control_loop)
        self.effector_timer = self.create_timer(
            0.5, self.publish_effector_parameters)
        self.cable_timer = self.create_timer(
            0.5, self.publish_cable_parameters)
        self.pulley_timer = self.create_timer(
            0.5, self.publish_pulley_parameters)
        self.get_logger().info('CONTROLADOR ACTIVADO. ESPERANDO COORDENADAS ...')

    def calculate_cable_parameters(self, x, y):
        left_pulley_x, left_pulley_y = 0.0, self.plane_height
        right_pulley_x, right_pulley_y = self.plane_width, self.plane_height
        left_connection_x = x - self.effector_width / 2.0
        left_connection_y = y + self.effector_height / 2.0
        right_connection_x = x + self.effector_width / 2.0
        right_connection_y = y + self.effector_height / 2.0
        left_len = math.sqrt((left_connection_x - left_pulley_x)
                             ** 2 + (left_connection_y - left_pulley_y)**2)
        right_len = math.sqrt((right_connection_x - right_pulley_x)
                              ** 2 + (right_connection_y - right_pulley_y)**2)
        left_ang = math.degrees(
            math.atan2(
                left_connection_x -
                left_pulley_x,
                left_pulley_y -
                left_connection_y))
        right_ang = math.degrees(
            math.atan2(
                right_pulley_x -
                right_connection_x,
                right_pulley_y -
                right_connection_y))
        return left_len, right_len, left_ang, right_ang

    def calculate_pulley_movement(self, start_x, start_y, end_x, end_y):
        l1_start, l2_start, _, _ = self.calculate_cable_parameters(
            start_x, start_y)
        l1_end, l2_end, _, _ = self.calculate_cable_parameters(end_x, end_y)
        delta_l1 = l1_end - l1_start
        delta_l2 = l2_end - l2_start
        pulley1_angle = delta_l1 / self.pulley_radius
        pulley2_angle = delta_l2 / self.pulley_radius
        return delta_l1, delta_l2, pulley1_angle, pulley2_angle

    def path_callback(self, msg: Path):
        self.points = []
        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            self.points.append((x, y))
        self.points_number = len(self.points)
        if self.points_number == 0:
            self.get_logger().error('ERROR. FORMATO DE MENSAJE INCORRECTO.')
            return
        if self.points_number == 1:
            self.mode = 'single'
            self.get_logger().info('UN PUNTO RECIBIDO (VERSION1_CONTROLLER). EJECUTANDO ...')
            self.handle_single_point()

        elif self.points_number == 2:
            self.mode = 'two_point'
            self.get_logger().info('DOS PUNTOS RECIBIDOS (VERSION2_CONTROLLER). EJECUTANDO ...')
            self.start_two_point_trajectory()
        elif self.points_number >= 3:
            self.mode = 'multi'
            self.get_logger().info(
                f'{self.points_number} PUNTOS RECIBIDOS (VERSION3_CONTROLLER). EJECUTANDO ...')
            self.start_multi_point_trajectory()
        self.marker_received = True

    def handle_single_point(self):
        x, y = self.points[0]
        self.current_x, self.current_y = x, y
        L1, L2, Q1, Q2 = self.calculate_cable_parameters(
            self.current_x, self.current_y)
        delta_l1, delta_l2, p1_ang, p2_ang = self.calculate_pulley_movement(
            self.plane_width / 2.0, self.plane_height / 2.0, self.current_x, self.current_y)
        path_msg = Path()
        ps = PoseStamped()
        ps.pose.position.x = self.current_x
        ps.pose.position.y = self.current_y
        path_msg.poses = [ps]
        self.effector_coordinates_publisher.publish(path_msg)
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
        print('=' * 100)
        print('POSICIÓN DEL EFECTOR FINAL (X, Y)')
        print(
            f'• POSICIÓN OBJETIVO: ({
                self.current_x:.3f}, {
                self.current_y:.3f}) m')
        print('LONGITUDES DE CADA CABLE (L1, L2)')
        print(f'• L1: {L1:.3f} m, L2: {L2:.3f} m')
        print('ÁNGULOS DE CADA CABLE (Q1, Q2)')
        print(f'• Q1: {Q1:.2f} °, Q2: {Q2:.2f} °')
        print('MOVIMIENTO DE CADA POLEA (P1, P2)')
        print(
            f'• CABLE ELONGADO / RECOGIDO:  P1: {delta_l1:+.3f} m,   P2: {delta_l2:+.3f} m')
        print(f'• ÁNGULOS DE GIRO (RADIANES): P1: {
              p1_ang:+.3f} rad, P2: {p2_ang:+.3f} rad')
        print('=' * 100)
        self.get_logger().info('CERRANDO CONTROLADOR ...')
        self.create_timer(0.05, self.shutdown_node)

    def start_two_point_trajectory(self):
        self.initial_x, self.initial_y = self.points[0]
        self.current_x, self.current_y = self.initial_x, self.initial_y
        self.target_x, self.target_y = self.points[1]
        self.moving = True
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        l1, l2, a1, a2 = self.calculate_cable_parameters(
            self.target_x, self.target_y)
        l1_init, l2_init, a1_init, a2_init = self.calculate_cable_parameters(
            self.initial_x, self.initial_y)
        self.target_left_cable_change = l1 - l1_init
        self.target_right_cable_change = l2 - l2_init
        self.target_left_pulley_angle = self.target_left_cable_change / self.pulley_radius
        self.target_right_pulley_angle = self.target_right_cable_change / self.pulley_radius
        self.get_logger().info(
            f'NUEVA TRAYECTORIA: ({
                self.initial_x:.3f}, {
                self.initial_y:.3f}) → ({
                self.target_x:.3f}, {
                    self.target_y:.3f})')

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
        self.get_logger().info(f'TRAYECTORIA CREADA: {self.points_number} puntos, {
            len(self.segment_target_params)} segmentos.')

    def control_loop(self):
        if self.objective_achieved and not self.shutdown_scheduled:
            self.shutdown_scheduled = True
            self.get_logger().info('CERRANDO CONTROLADOR ...')
            self.create_timer(0.05, self.shutdown_node)
            return
        if self.moving and not self.objective_achieved:
            self.animation_progress += 0.02
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
                    self.get_logger().info(
                        f'DESPLAZAMIENTO DESDE EL PUNTO {
                            self.segment_index +
                            1} HASTA EL PUNTO {
                            self.segment_index +
                            2}')
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

    def publish_effector_parameters(self):
        path_msg = Path()
        pose_current = PoseStamped()
        pose_current.pose.position.x = self.current_x
        pose_current.pose.position.y = self.current_y
        pose_target = PoseStamped()
        pose_target.pose.position.x = self.target_x
        pose_target.pose.position.y = self.target_y
        path_msg.poses = [pose_current, pose_target]
        self.effector_coordinates_publisher.publish(path_msg)
        if self.moving:
            progress_percent = self.animation_progress * 100.0
            if self.mode == 'multi':
                self.get_logger().info(
                    f'PROGRESO [{self.segment_index + 1}/{max(1, self.points_number - 1)}] '
                    f'{progress_percent:.1f}% ACTUAL=({self.current_x:.3f}, {self.current_y:.3f}) '
                    f'OBJETIVO=({self.target_x:.3f}, {self.target_y:.3f})'
                )
            else:
                self.get_logger().info(
                    f'PROGRESO {
                        progress_percent:.1f}% - ACTUAL=({
                        self.current_x:.3f}, {
                        self.current_y:.3f}) OBJETIVO=({
                        self.target_x:.3f}, {
                        self.target_y:.3f})')
        else:
            self.get_logger().info(
                f'EFECTOR: EN REPOSO - POSICIÓN=({self.current_x:.3f}, {self.current_y:.3f})')

    def publish_cable_parameters(self):
        L1, L2, Q1, Q2 = self.calculate_cable_parameters(
            self.current_x, self.current_y)
        msg = Float32MultiArray()
        msg.data = [L1, L2, Q1, Q2]
        self.cable_parameters_publisher.publish(msg)
        self.get_logger().info(
            f'CABLES: (L1, L2)=({
                L1:.3f}, {
                L2:.3f}) m, (Q1, Q2)=({
                Q1:.2f}, {
                    Q2:.2f})°')

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
        if self.mode == 'multi':
            self.get_logger().info(
                f'POLEAS: SEGMENTO [{self.segment_index + 1}/{max(1, self.points_number - 1)}] '
                f'DIF=({dl1:+.3f} m, {dl2:+.3f} m), ÁNGULOS=({pa1:+.3f} rad, {pa2:+.3f} rad)'
            )
        else:
            self.get_logger().info(f'POLEAS: DIF=({
                dl1:+.3f} m, {dl2:+.3f} m), ÁNGULOS=({pa1:+.3f} rad, {pa2:+.3f} rad)')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pulley_position, self.right_pulley_position]
        self.joint_state_publisher.publish(msg)

    def show_final_summary(self):
        if self.mode == 'two_point':
            final_left_length, final_right_length, final_left_angle, final_right_angle = \
                self.calculate_cable_parameters(self.current_x, self.current_y)
            initial_left_length, initial_right_length, initial_left_angle, initial_right_angle = \
                self.calculate_cable_parameters(self.initial_x, self.initial_y)
            print('═' * 100)
            print('POSICIONES DEL EFECTOR FINAL (X, Y)')
            print(
                f'• POSICIÓN INICIAL:  ({
                    self.initial_x:.3f}, {
                    self.initial_y:.3f}) m')
            print(
                f'• POSICIÓN FINAL:    ({
                    self.current_x:.3f}, {
                    self.current_y:.3f}) m')
            print(
                f'• POSICIÓN OBJETIVO: ({
                    self.target_x:.3f}, {
                    self.target_y:.3f}) m')
            print('')
            print('LONGITUDES DE CADA CABLE (L1, L2)')
            print(
                f'• LONGITUDES INICIALES:     L1: {
                    initial_left_length:.3f} m, L2: {
                    initial_right_length:.3f} m')
            print(
                f'• LONGITUDES FINALES:       L1: {
                    final_left_length:.3f} m, L2: {
                    final_right_length:.3f} m')
            print(f'• DIFERENCIA DE LONGITUDES: L1: {self.target_left_cable_change:+.3f} m, '
                  f'L2: {self.target_right_cable_change:+.3f} m')
            print('')
            print('ÁNGULOS DE CADA CABLE (Q1, Q2)')
            print(
                f'• ÁNGULOS INICIALES:  Q1: {
                    initial_left_angle:.2f} °, Q2: {
                    initial_right_angle:.2f} °')
            print(
                f'• ÁNGULOS FINALES: Q1: {
                    final_left_angle:.2f} °, Q2: {
                    final_right_angle:.2f} °')
            print('')
            print('MOVIMIENTO DE CADA POLEA (P1, P2)')
            print(
                f'• CABLE ELONGADO / RECOGIDO POR P1: {self.target_left_cable_change:+.3f} m')
            print(
                f'• CABLE ELONGADO / RECOGIDO POR P2: {self.target_right_cable_change:+.3f} m')
            print(
                f'• ÁNGULO DE GIRO DE P1: {
                    self.target_left_pulley_angle:+.3f} rad ({
                    math.degrees(
                        self.target_left_pulley_angle):+.2f} °)')
            print(
                f'• ÁNGULO DE GIRO DE P2: {
                    self.target_right_pulley_angle:+.3f} rad ({
                    math.degrees(
                        self.target_right_pulley_angle):+.2f} °)')
            print('═' * 100)
        elif self.mode == 'multi':
            print('═' * 100)
            print(f'NÚMERO DE PUNTOS: {self.points_number}')
            print('\nPUNTOS POR LOS QUE HA PASADO EL EFECTOR')
            for i, (x, y) in enumerate(self.points):
                print(f'• {i + 1:02d}: ({x:.3f}, {y:.3f}) m')
            for i, (x, y) in enumerate(self.points):
                L1, L2, q1, q2 = self.calculate_cable_parameters(x, y)
                print(f'\nDATOS POSICIÓN NÚMERO {i + 1}')
                print(f'• L1 = {L1:.3f} m, L2 = {L2:.3f} m')
                print(f'• Q1 = {q1:.2f} °, Q2 = {q2:.2f} °')
            for i in range(self.points_number - 1):
                sx, sy = self.points[i]
                ex, ey = self.points[i + 1]
                L1_i, L2_i, _, _ = self.calculate_cable_parameters(sx, sy)
                L1_f, L2_f, _, _ = self.calculate_cable_parameters(ex, ey)
                L1_mov = L1_f - L1_i
                L2_mov = L2_f - L2_i
                P1_mov_rad = L1_mov / self.pulley_radius
                P2_mov_rad = L2_mov / self.pulley_radius
                print(f'\nMOVIMIENTO ENTRE POSICIONES {i + 1} Y {i + 2}')
                print(f'• ΔL1 = {L1_mov:+.3f} m, ΔL2 = {L2_mov:+.3f} m')
                print(
                    f'• P1 = {P1_mov_rad:+.3f} rad ({math.degrees(P1_mov_rad):+.2f} °)')
                print(
                    f'• P2 = {P2_mov_rad:+.3f} rad ({math.degrees(P2_mov_rad):+.2f} °)')
            print('═' * 100)
        elif self.mode == 'single':
            pass

    def shutdown_node(self):
        try:
            self.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = CDPRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\nCERRANDO CONTROLADOR ...')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            if rclpy.ok():
                controller.destroy_node()
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
