import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster

# COMPILING, LINKING, AND EXECUTING
# TERMINAL 1: colcon build --symlink-install
# TERMINAL 2: ros2 run cdpr_2d version2_controller
# TERMINAL 3: ros2 topic echo /effector_coordinates
# TERMINAL 4: ros2 topic echo /cable_parameters
# TERMINAL 5: ros2 topic echo /pulley_parameters
# TERMINAL 6: ros2 topic pub --once /version2 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.3, y: 0.3, z: 0.0}}}"
# TERMINAL 6: ros2 topic pub --once /version2
# geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose:
# {position: {x: 0.7, y: 0.7, z: 0.0}}}"


class Version2Controller(Node):

    def __init__(self):
        super().__init__('version2_controller')
        self.plane_width = 1.0
        self.plane_height = 1.0
        self.effector_width = 0.1
        self.effector_height = 0.1
        self.pulley_radius = 0.05
        self.current_x = 0.5
        self.current_y = 0.5
        self.target_x = 0.5
        self.target_y = 0.5
        self.initial_x = 0.5
        self.initial_y = 0.5
        self.moving = False
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        self.target_left_cable_change = 0.0
        self.target_right_cable_change = 0.0
        self.target_left_pulley_angle = 0.0
        self.target_right_pulley_angle = 0.0
        self.received_first_pose = False
        self.first_pose = None
        self.control_timer = self.create_timer(0.5, self.control_loop)
        self.effector_timer = self.create_timer(
            0.5, self.publish_effector_parameters)
        self.cable_timer = self.create_timer(
            0.5, self.publish_cable_parameters)
        self.pulley_timer = self.create_timer(
            0.5, self.publish_pulley_parameters)
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
            PoseStamped, '/version2', self.target_coordinates_callback, qos_profile)
        self.get_logger().info("CONTROLADOR ACTIVADO. ESPERANDO COORDENADAS ...")

    def target_coordinates_callback(self, msg: PoseStamped):
        if not self.received_first_pose:
            self.first_pose = msg.pose
            self.received_first_pose = True
            self.get_logger().info(
                f"POSICIÓN INICIAL RECIBIDA: ({
                    msg.pose.position.x:.3f}, {
                    msg.pose.position.y:.3f})")
            return
        start = self.first_pose.position
        end = msg.pose.position
        self.initial_x = start.x
        self.initial_y = start.y
        self.current_x = start.x
        self.current_y = start.y
        self.target_x = end.x
        self.target_y = end.y
        self.moving = True
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        self.received_first_pose = False
        self.calculate_target_parameters()
        self.get_logger().info(
            f"POSICIÓN FINAL RECIBIDA: ({
                self.target_x:.3f}, {
                self.target_y:.3f})")
        self.get_logger().info(
            f"NUEVA TRAYECTORIA: ({
                self.initial_x:.3f}, {
                self.initial_y:.3f}) → ({
                self.target_x:.3f}, {
                    self.target_y:.3f})")

    def calculate_target_parameters(self):
        l1, l2, a1, a2 = self.calculate_cable_parameters(
            self.target_x, self.target_y)
        l1_init, l2_init, a1_init, a2_init = self.calculate_cable_parameters(
            self.initial_x, self.initial_y)
        self.target_left_cable_change = l1 - l1_init
        self.target_right_cable_change = l2 - l2_init
        self.target_left_pulley_angle = self.target_left_cable_change / self.pulley_radius
        self.target_right_pulley_angle = self.target_right_cable_change / self.pulley_radius

    def calculate_cable_parameters(self, x, y):
        left_pulley_x, left_pulley_y = 0.0, self.plane_height
        right_pulley_x, right_pulley_y = self.plane_width, self.plane_height
        left_conn_x = x - self.effector_width / 2
        left_conn_y = y + self.effector_height / 2
        right_conn_x = x + self.effector_width / 2
        right_conn_y = y + self.effector_height / 2
        left_len = math.sqrt((left_conn_x - left_pulley_x)
                             ** 2 + (left_conn_y - left_pulley_y)**2)
        right_len = math.sqrt((right_conn_x - right_pulley_x)
                              ** 2 + (right_conn_y - right_pulley_y)**2)
        left_ang = math.degrees(
            math.atan2(
                left_conn_x -
                left_pulley_x,
                left_pulley_y -
                left_conn_y))
        right_ang = math.degrees(
            math.atan2(
                right_pulley_x -
                right_conn_x,
                right_pulley_y -
                right_conn_y))
        return left_len, right_len, left_ang, right_ang

    def calculate_pulley_movement(self, x1, y1, x2, y2):
        l1_start, l2_start, *_ = self.calculate_cable_parameters(x1, y1)
        l1_end, l2_end, *_ = self.calculate_cable_parameters(x2, y2)
        d1 = l1_end - l1_start
        d2 = l2_end - l2_start
        return d1, d2, d1 / self.pulley_radius, d2 / self.pulley_radius

    def control_loop(self):
        if self.objective_achieved and not self.shutdown_scheduled:
            self.shutdown_scheduled = True
            self.get_logger().info("CERRANDO CONTROLADOR ...")
            self.create_timer(0.05, self.shutdown_node)
            return
        if self.moving and not self.objective_achieved:
            self.animation_progress += 0.02
            if self.animation_progress >= 1.0:
                self.animation_progress = 1.0
                self.moving = False
                self.current_x = self.target_x
                self.current_y = self.target_y
                if self.check_objective_achieved():
                    self.objective_achieved = True
                    self.show_final_summary()
                return
            new_x = self.initial_x + \
                (self.target_x - self.initial_x) * self.animation_progress
            new_y = self.initial_y + \
                (self.target_y - self.initial_y) * self.animation_progress
            d1, d2, a1, a2 = self.calculate_pulley_movement(
                self.current_x, self.current_y, new_x, new_y)
            self.left_pulley_position += a1
            self.right_pulley_position += a2
            self.current_x = new_x
            self.current_y = new_y
            self.publish_joint_states()

    def check_objective_achieved(self):
        return abs(
            self.current_x -
            self.target_x) < 0.005 and abs(
            self.current_y -
            self.target_y) < 0.005

    def publish_effector_parameters(self):
        if self.objective_achieved:
            return
        path_msg = Path()
        current_pose = PoseStamped()
        current_pose.pose.position.x = self.current_x
        current_pose.pose.position.y = self.current_y
        target_pose = PoseStamped()
        target_pose.pose.position.x = self.target_x
        target_pose.pose.position.y = self.target_y
        path_msg.poses = [current_pose, target_pose]
        self.effector_coordinates_publisher.publish(path_msg)
        if self.moving:
            progress_percent = self.animation_progress * 100
            self.get_logger().info(
                f"EFECTOR: PROGRESO {
                    progress_percent:.1f}% - ACTUAL = ({
                    self.current_x:.3f}, {
                    self.current_y:.3f}) OBJETIVO = ({
                    self.target_x:.3f}, {
                        self.target_y:.3f})")
        else:
            self.get_logger().info(
                f"EFECTOR: EN REPOSO - POSICIÓN = ({self.current_x:.3f}, {self.current_y:.3f})")

    def publish_cable_parameters(self):
        if self.objective_achieved:
            return
        l1, l2, a1, a2 = self.calculate_cable_parameters(
            self.current_x, self.current_y)
        msg = Float32MultiArray()
        msg.data = [l1, l2, a1, a2]
        self.cable_parameters_publisher.publish(msg)
        self.get_logger().info(
            f"CABLES: L=({
                l1:.3f}, {
                l2:.3f}) m, ÁNGULOS=({
                a1:.2f}, {
                    a2:.2f})°")

    def publish_pulley_parameters(self):
        if self.objective_achieved:
            return
        if self.moving:
            d1, d2, a1, a2 = self.calculate_pulley_movement(
                self.initial_x, self.initial_y, self.current_x, self.current_y)
        else:
            d1 = d2 = a1 = a2 = 0.0
        msg = Float32MultiArray()
        msg.data = [d1, d2, a1, a2]
        self.pulley_parameters_publisher.publish(msg)
        self.get_logger().info(
            f"POLEAS: ΔL=({d1:+.3f}, {d2:+.3f}) m, ÁNGULOS=({a1:+.3f}, {a2:+.3f}) rad")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_pulley_position, self.right_pulley_position]
        self.joint_state_publisher.publish(msg)

    def show_final_summary(self):
        final_left_length, final_right_length, final_left_angle, final_right_angle = self.calculate_cable_parameters(
            self.current_x, self.current_y)
        initial_left_length, initial_right_length, initial_left_angle, initial_right_angle = self.calculate_cable_parameters(
            self.initial_x, self.initial_y)
        print("═" * 100)
        print("POSICIONES DEL EFECTOR FINAL (X, Y)")
        print(
            f"• POSICIÓN INICIAL:  ({
                self.initial_x:.3f}, {
                self.initial_y:.3f}) m")
        print(
            f"• POSICIÓN FINAL:    ({
                self.current_x:.3f}, {
                self.current_y:.3f}) m")
        print(
            f"• POSICIÓN OBJETIVO: ({
                self.target_x:.3f}, {
                self.target_y:.3f}) m")
        print("")
        print("LONGITUDES DE CADA CABLE (L1, L2)")
        print(
            f"• LONGITUDES INICIALES:     L1: {
                initial_left_length:.3f} m, L2: {
                initial_right_length:.3f} m")
        print(
            f"• LONGITUDES FINALES:       L1: {
                final_left_length:.3f} m, L2: {
                final_right_length:.3f} m")
        print(
            f"• DIFERENCIA DE LONGITUDES: L1: {
                self.target_left_cable_change:+.3f} m, L2: {
                self.target_right_cable_change:+.3f} m")
        print("")
        print("ÁNGULOS DE CADA CABLE (Q1, Q2)")
        print(
            f"• ÁNGULOS INICIALES:  Q1: {
                initial_left_angle:.2f} °, Q2: {
                initial_right_angle:.2f} °")
        print(
            f"• ÁNGULOS FINALES: Q1: {
                final_left_angle:.2f} °, Q2: {
                final_right_angle:.2f} °")
        print("")
        print("MOVIMIENTO DE CADA POLEA (P1, P2)")
        print(
            f"• CABLE ELONGADO / RECOGIDO POR P1: {self.target_left_cable_change:+.3f} m")
        print(
            f"• CABLE ELONGADO / RECOGIDO POR P2: {self.target_right_cable_change:+.3f} m")
        print(
            f"• ÁNGULO DE GIRO DE P1: {
                self.target_left_pulley_angle:+.3f} rad ({
                math.degrees(
                    self.target_left_pulley_angle):+.2f} °)")
        print(
            f"• ÁNGULO DE GIRO DE P2: {
                self.target_right_pulley_angle:+.3f} rad ({
                math.degrees(
                    self.target_right_pulley_angle):+.2f} °)")
        print("═" * 100)

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = Version2Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nCERRANDO CONTROLADOR ...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            if rclpy.ok():
                controller.destroy_node()
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
