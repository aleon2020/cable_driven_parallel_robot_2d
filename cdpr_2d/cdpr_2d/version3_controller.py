import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster

# COMPILING, LINKING, AND EXECUTING
# TERMINAL 1: colcon build --symlink-install
# TERMINAL 2: ros2 run cdpr_2d version3_controller
# TERMINAL 3: ros2 topic echo /effector_coordinates
# TERMINAL 4: ros2 topic echo /cable_parameters
# TERMINAL 5: ros2 topic echo /pulley_parameters
# TERMINAL 6: ros2 topic pub --once /version3 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: 0.2, z: 0.0}}}"
# TERMINAL 6: ros2 topic pub --once /version3 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: 0.8, z: 0.0}}}"
# TERMINAL 6: ros2 topic pub --once /version3 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.8, y: 0.8, z: 0.0}}}"
# TERMINAL 6: ros2 topic pub --once /version3 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.8, y: 0.2, z: 0.0}}}"
# TERMINAL 6: ros2 topic pub --once /version3 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.2, y: 0.2, z: 0.0}}}"
# TERMINAL 6: ros2 topic pub --once /version3 geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: -1.0, y: -1.0, z: 0.0}}}"

class Version3Controller(Node):

    def __init__(self):
        super().__init__('version3_controller')
        self.plane_width = 1.0
        self.plane_height = 1.0
        self.effector_width = 0.1
        self.effector_height = 0.1
        self.pulley_radius = 0.05
        self.current_x = 0.5
        self.current_y = 0.5
        self.target_x = self.current_x
        self.target_y = self.current_y
        self.points = []
        self.points_number = 0
        self.segment_index = 0
        self.moving = False
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.left_pulley_position = 0.0
        self.right_pulley_position = 0.0
        self.initial_x = self.current_x
        self.initial_y = self.current_y
        self.segment_target_params = []
        qos_profile = QoSProfile(depth=10)
        self.effector_coordinates_publisher = self.create_publisher(Path, '/effector_coordinates', qos_profile)
        self.cable_parameters_publisher = self.create_publisher(Float32MultiArray, '/cable_parameters', qos_profile)
        self.pulley_parameters_publisher = self.create_publisher(Float32MultiArray, '/pulley_parameters', qos_profile)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.coordinates_subscriber = self.create_subscription(PoseStamped, '/version3', self.pose_callback, qos_profile)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.effector_timer = self.create_timer(0.25, self.publish_effector_parameters)
        self.cable_timer = self.create_timer(0.25, self.publish_cable_parameters)
        self.pulley_timer = self.create_timer(0.25, self.publish_pulley_parameters)
        self.get_logger().info("CONTROLADOR ACTIVADO. ESPERANDO COORDENADAS ...")

    def calculate_cable_parameters(self, x, y):
        left_pulley_x, left_pulley_y = 0.0, self.plane_height
        right_pulley_x, right_pulley_y = self.plane_width, self.plane_height
        left_connection_x = x - self.effector_width / 2
        left_connection_y = y + self.effector_height / 2
        right_connection_x = x + self.effector_width / 2
        right_connection_y = y + self.effector_height / 2
        left_cable_length = math.sqrt((left_connection_x - left_pulley_x)**2 + (left_connection_y - left_pulley_y)**2)
        right_cable_length = math.sqrt((right_connection_x - right_pulley_x)**2 + (right_connection_y - right_pulley_y)**2)
        left_cable_angle = math.degrees(math.atan2(left_connection_x - left_pulley_x, left_pulley_y - left_connection_y))
        right_cable_angle = math.degrees(math.atan2(right_pulley_x - right_connection_x, right_pulley_y - right_connection_y))
        return left_cable_length, right_cable_length, left_cable_angle, right_cable_angle

    def calculate_pulley_movement(self, start_x, start_y, end_x, end_y):
        l1_start, l2_start, _, _ = self.calculate_cable_parameters(start_x, start_y)
        l1_end, l2_end, _, _ = self.calculate_cable_parameters(end_x, end_y)
        delta_l1 = l1_end - l1_start
        delta_l2 = l2_end - l2_start
        pulley1_angle = delta_l1 / self.pulley_radius
        pulley2_angle = delta_l2 / self.pulley_radius
        return delta_l1, delta_l2, pulley1_angle, pulley2_angle

    def check_objective_achieved(self):
        return abs(self.current_x - self.target_x) < 0.005 and abs(self.current_y - self.target_y) < 0.005

    def pose_callback(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        if x == -1.0 and y == -1.0:
            if len(self.points) > 2:
                self.start_trajectory()
                self.get_logger().info(f"TRAYECTORIA OBTENIDA DE {len(self.points)} PUNTOS. INICIANDO MOVIMIENTO ...")
            else:
                self.get_logger().error("SE REQUIEREN AL MENOS TRES PUNTOS")
                self.create_timer(3.0, self.shutdown_node)
                return
            return
        self.points.append((x, y))
        self.get_logger().info(f"PUNTO {len(self.points)} RECIBIDO: ({x:.3f}, {y:.3f})")

    def start_trajectory(self):
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
            delta_l1, delta_l2, p1_angle, p2_angle = self.calculate_pulley_movement(sx, sy, ex, ey)
            self.segment_target_params.append({
                'start': (sx, sy),
                'end': (ex, ey),
                'end_lengths': (l1, l2),
                'delta_l': (delta_l1, delta_l2),
                'pulley_angles': (p1_angle, p2_angle)
            })

    def control_loop(self):
        if self.objective_achieved and not self.shutdown_scheduled:
            self.shutdown_scheduled = True
            self.get_logger().info("CERRANDO CONTROLADOR ...")
            self.create_timer(3.0, self.shutdown_node)
            return
        if self.moving and not self.objective_achieved:
            self.animation_progress += 0.02
            if self.animation_progress >= 1.0:
                self.animation_progress = 1.0
                self.current_x = self.target_x
                self.current_y = self.target_y
                delta_l1, delta_l2, pulley1_angle, pulley2_angle = self.calculate_pulley_movement(self.initial_x, self.initial_y, self.current_x, self.current_y)
                self.left_pulley_position += pulley1_angle
                self.right_pulley_position += pulley2_angle
                self.publish_joint_states()
                if self.segment_index < (self.points_number - 2):
                    self.segment_index += 1
                    self.initial_x, self.initial_y = self.points[self.segment_index]
                    self.target_x, self.target_y = self.points[self.segment_index + 1]
                    self.animation_progress = 0.0
                    self.get_logger().info(f"DESPLAZAMIENTO DESDE EL PUNTO {self.segment_index+1} HASTA EL PUNTO {self.segment_index+2}")
                else:
                    self.objective_achieved = True
                    self.moving = False
                    self.show_final_summary()
                return
            interpolated_x = self.initial_x + (self.target_x - self.initial_x) * self.animation_progress
            interpolated_y = self.initial_y + (self.target_y - self.initial_y) * self.animation_progress
            delta_l1, delta_l2, pulley1_angle, pulley2_angle = self.calculate_pulley_movement(self.current_x, self.current_y, interpolated_x, interpolated_y)
            self.left_pulley_position += pulley1_angle
            self.right_pulley_position += pulley2_angle
            self.publish_joint_states()
            self.current_x = interpolated_x
            self.current_y = interpolated_y

    def publish_effector_parameters(self):
        path_msg = Path()
        pose_current = PoseStamped()
        pose_target = PoseStamped()
        pose_current.pose.position.x = self.current_x
        pose_current.pose.position.y = self.current_y
        pose_target.pose.position.x = self.target_x
        pose_target.pose.position.y = self.target_y
        path_msg.poses = [pose_current, pose_target]
        self.effector_coordinates_publisher.publish(path_msg)
        if self.moving:
            progress_percent = self.animation_progress * 100
            self.get_logger().info(
                f"EFECTOR: PROGRESO [{self.segment_index+1}/{max(1,self.points_number-1)}] "
                f"{progress_percent:.1f}% - POSICIÓN ACTUAL = ({self.current_x:.3f}, {self.current_y:.3f}), "
                f"OBJETIVO = ({self.target_x:.3f}, {self.target_y:.3f})"
            )
        else:
            self.get_logger().info(f"EFECTOR: EN REPOSO - POSICIÓN = ({self.current_x:.3f}, {self.current_y:.3f})")

    def publish_cable_parameters(self):
        L1, L2, Q1, Q2 = self.calculate_cable_parameters(self.current_x, self.current_y)
        msg = Float32MultiArray()
        msg.data = [L1, L2, Q1, Q2]
        self.cable_parameters_publisher.publish(msg)
        self.get_logger().info(
            f"CABLES: (L1, L2)=({L1:.3f} m, {L2:.3f} m), "
            f"(Q1, Q2)=({Q1:.2f}°, {Q2:.2f}°)"
        )

    def publish_pulley_parameters(self):
        if self.moving:
            seg_start_x, seg_start_y = self.initial_x, self.initial_y
            dl1, dl2, pa1, pa2 = self.calculate_pulley_movement(seg_start_x, seg_start_y, self.current_x, self.current_y)
        else:
            dl1 = dl2 = pa1 = pa2 = 0.0
        msg = Float32MultiArray()
        msg.data = [dl1, dl2, pa1, pa2]
        self.pulley_parameters_publisher.publish(msg)
        self.get_logger().info(
            f"POLEAS: SEGMENTO [{self.segment_index+1}/{max(1,self.points_number-1)}] "
            f"DIF=({dl1:+.3f} m, {dl2:+.3f} m), ÁNGULOS=({pa1:+.3f} rad, {pa2:+.3f} rad)"
        )

    def publish_joint_states(self):
        joint_state_message = JointState()
        joint_state_message.header.stamp = self.get_clock().now().to_msg()
        joint_state_message.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_message.position = [self.left_pulley_position, self.right_pulley_position]
        self.joint_state_publisher.publish(joint_state_message)

    def show_final_summary(self):
        print("═" * 100)
        print(f"NÚMERO DE PUNTOS: {self.points_number}")
        print("\nPUNTOS POR LOS QUE HA PASADO EL EFECTOR")
        for i, (x, y) in enumerate(self.points):
            print(f"• {i+1:02d}: ({x:.3f}, {y:.3f}) m")
        for i, (x, y) in enumerate(self.points):
            L1, L2, q1, q2 = self.calculate_cable_parameters(x, y)
            print(f"\nDATOS POSICIÓN NÚMERO {i+1}")
            print(f"• L1 = {L1:.3f} m, L2 = {L2:.3f} m")
            print(f"• Q1 = {q1:.2f} °, Q2 = {q2:.2f} °")
        for i in range(self.points_number - 1):
            sx, sy = self.points[i]
            ex, ey = self.points[i+1]
            L1_i, L2_i, _, _ = self.calculate_cable_parameters(sx, sy)
            L1_f, L2_f, _, _ = self.calculate_cable_parameters(ex, ey)
            L1_mov = L1_f - L1_i
            L2_mov = L2_f - L2_i
            P1_mov_rad = L1_mov / self.pulley_radius
            P2_mov_rad = L2_mov / self.pulley_radius
            print(f"\nMOVIMIENTO ENTRE POSICIONES {i+1} Y {i+2}")
            print(f"• ΔL1 = {L1_mov:+.3f} m, ΔL2 = {L2_mov:+.3f} m")
            print(f"• P1 = {P1_mov_rad:+.3f} rad ({math.degrees(P1_mov_rad):+.2f} °)")
            print(f"• P2 = {P2_mov_rad:+.3f} rad ({math.degrees(P2_mov_rad):+.2f} °)")
        print("═" * 100)

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = Version3Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nCERRANDO CONTROLADOR ...")
    finally:
        if rclpy.ok():
            controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()