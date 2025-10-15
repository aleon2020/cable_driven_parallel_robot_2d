import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import numpy as np
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, TransformStamped
from interfaces_package.msg import EffectorCoordinates, CableParameters, PulleyParameters
import sys

class CableRobotController(Node):

    def __init__(self):
        super().__init__('cable_robot_controller')
        self.platform_width = 1.0
        self.platform_height = 1.0  
        self.effector_width = 0.1
        self.effector_height = 0.1
        self.pulley_radius = 0.05
        self.current_x = 0.5
        self.current_y = 0.5
        self.target_x = 0.5
        self.target_y = 0.5
        self.moving = False
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.shutdown_scheduled = False
        self.initial_x = self.current_x
        self.initial_y = self.current_y
        self.target_left_length = 0.0
        self.target_right_length = 0.0
        self.target_left_angle = 0.0
        self.target_right_angle = 0.0
        self.target_left_cable_change = 0.0
        self.target_right_cable_change = 0.0
        self.target_left_pulley_angle = 0.0
        self.target_right_pulley_angle = 0.0
        self.left_pulley_pos = 0.0
        self.right_pulley_pos = 0.0
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.effector_timer = self.create_timer(0.25, self.publish_effector_parameters)
        self.cable_timer = self.create_timer(0.25, self.publish_cable_parameters)
        self.pulley_timer = self.create_timer(0.25, self.publish_pulley_parameters)
        qos_profile = QoSProfile(depth=10)
        self.effector_coords_pub = self.create_publisher(EffectorCoordinates, '/effector_coordinates', qos_profile)
        self.cable_params_pub = self.create_publisher(CableParameters, '/cable_parameters', qos_profile)
        self.pulley_params_pub = self.create_publisher(PulleyParameters, '/pulley_parameters', qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.effector_log_pub = self.create_publisher(String, '/effector_log', qos_profile)
        self.cable_log_pub = self.create_publisher(String, '/cable_log', qos_profile)
        self.pulley_log_pub = self.create_publisher(String, '/pulley_log', qos_profile)
        self.coords_sub = self.create_subscription(
            EffectorCoordinates,
            '/target_coordinates',
            self.target_coords_callback,
            qos_profile
        )
        self._calculate_target_parameters()
        self.get_logger().info("CONTROLADOR ACTIVADO. ESPERANDO COORDENADAS ...")

    def _calculate_target_parameters(self):
        (self.target_left_length, self.target_right_length, self.target_left_angle, self.target_right_angle) = self.calculate_cable_parameters(self.target_x, self.target_y)
        (self.target_left_cable_change, self.target_right_cable_change, self.target_left_pulley_angle, self.target_right_pulley_angle) = self.calculate_pulley_movement(self.initial_x, self.initial_y, self.target_x, self.target_y)

    def target_coords_callback(self, msg):
        if self.objective_achieved:
            self.get_logger().info("OBJETIVO CUMPLIDO")
            return
        self.initial_x = msg.current.x
        self.initial_y = msg.current.y
        self.current_x = msg.current.x
        self.current_y = msg.current.y
        self.target_x = msg.target.x
        self.target_y = msg.target.y
        self.moving = True
        self.animation_progress = 0.0
        self.objective_achieved = False
        self.left_pulley_pos = 0.0
        self.right_pulley_pos = 0.0
        self._calculate_target_parameters()
        self.get_logger().info(f"NUEVA TRAYECTORIA DESDE ({self.initial_x:.3f}, {self.initial_y:.3f}) HASTA ({self.target_x:.3f}, {self.target_y:.3f})")

    def calculate_cable_parameters(self, x, y):
        left_pulley_x = 0.0
        left_pulley_y = self.platform_height
        right_pulley_x = self.platform_width
        right_pulley_y = self.platform_height
        left_connection_x = x - self.effector_width/2
        left_connection_y = y + self.effector_height/2
        right_connection_x = x + self.effector_width/2
        right_connection_y = y + self.effector_height/2
        left_cable_length = math.sqrt((left_connection_x - left_pulley_x)**2 + (left_connection_y - left_pulley_y)**2)
        right_cable_length = math.sqrt((right_connection_x - right_pulley_x)**2 + (right_connection_y - right_pulley_y)**2)
        left_cable_angle = math.degrees(math.atan2(left_connection_x - left_pulley_x, left_pulley_y - left_connection_y))
        right_cable_angle = math.degrees(math.atan2(right_pulley_x - right_connection_x, right_pulley_y - right_connection_y))
        return left_cable_length, right_cable_length, left_cable_angle, right_cable_angle

    def calculate_pulley_movement(self, start_x, start_y, end_x, end_y):
        l1_start, l2_start, a1_start, a2_start = self.calculate_cable_parameters(start_x, start_y)
        l1_end, l2_end, a1_end, a2_end = self.calculate_cable_parameters(end_x, end_y)
        delta_l1 = l1_end - l1_start
        delta_l2 = l2_end - l2_start
        pulley1_angle = delta_l1 / self.pulley_radius
        pulley2_angle = delta_l2 / self.pulley_radius
        return delta_l1, delta_l2, pulley1_angle, pulley2_angle

    def check_objective_achieved(self):
        position_tolerance = 0.005
        position_ok = (abs(self.current_x - self.target_x) < position_tolerance and abs(self.current_y - self.target_y) < position_tolerance)
        return position_ok

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
                self.moving = False
                self.current_x = self.target_x
                self.current_y = self.target_y
                if self.check_objective_achieved():
                    self.objective_achieved = True
                    self.show_final_summary()
                else:
                    self.get_logger().info("TRAYECTORIA COMPLETADA")
            interp_x = self.initial_x + (self.target_x - self.initial_x) * self.animation_progress
            interp_y = self.initial_y + (self.target_y - self.initial_y) * self.animation_progress
            if self.animation_progress == 0.02:
                delta_l1, delta_l2, pulley1_angle, pulley2_angle = self.calculate_pulley_movement(self.initial_x, self.initial_y, interp_x, interp_y)
            else:
                delta_l1, delta_l2, pulley1_angle, pulley2_angle = self.calculate_pulley_movement(self.current_x, self.current_y, interp_x, interp_y)
            self.left_pulley_pos += pulley1_angle
            self.right_pulley_pos += pulley2_angle
            self.publish_joint_states()
            self.current_x = interp_x
            self.current_y = interp_y

    def show_final_summary(self):
        final_left_length, final_right_length, final_left_angle, final_right_angle = self.calculate_cable_parameters(self.current_x, self.current_y)
        initial_left_length, initial_right_length, initial_left_angle, initial_right_angle = self.calculate_cable_parameters(self.initial_x, self.initial_y)
        self.publish_final_status()
        print("═" * 50)
        print("DATOS OBTENIDOS")
        print("POSICIONES DEL EFECTOR FINAL (X, Y)")
        print(f"• POSICIÓN INICIAL:  ({self.initial_x:.3f}, {self.initial_y:.3f}) m")
        print(f"• POSICIÓN FINAL:    ({self.current_x:.3f}, {self.current_y:.3f}) m")
        print(f"• POSICIÓN OBJETIVO: ({self.target_x:.3f}, {self.target_y:.3f}) m")
        print("")
        print("LONGITUDES DE CADA CABLE (L1, L2)")
        print(f"• LONGITUDES INICIALES:     L1: {initial_left_length:.3f} m, L2: {initial_right_length:.3f} m")
        print(f"• LONGITUDES FINALES:       L1: {final_left_length:.3f} m, L2: {final_right_length:.3f} m")
        print(f"• DIFERENCIA DE LONGITUDES: L1: {self.target_left_cable_change:+.3f} m, L2: {self.target_right_cable_change:+.3f} m")
        print("")
        print("ÁNGULOS DE CADA CABLE (Q1, Q2)")
        print(f"• ÁNGULOS INICIALES:  Q1: {initial_left_angle:.2f}°, Q2: {initial_right_angle:.2f}°")
        print(f"• ÁNGULOS FINALES:    Q1: {final_left_angle:.2f}°, Q2: {final_right_angle:.2f}°")
        print("")
        print("MOVIMIENTO DE CADA POLEA (P1, P2)")
        print(f"• CABLE ELONGADO / RECOGIDO POR P1: {self.target_left_cable_change:+.3f} m")
        print(f"• CABLE ELONGADO / RECOGIDO POR P2: {self.target_right_cable_change:+.3f} m")
        print(f"• ÁNGULO DE GIRO DE P1: {self.target_left_pulley_angle:+.3f} rad ({math.degrees(self.target_left_pulley_angle):+.2f} °)")
        print(f"• ÁNGULO DE GIRO DE P2: {self.target_right_pulley_angle:+.3f} rad ({math.degrees(self.target_right_pulley_angle):+.2f} °)")
        print("═" * 50)
        print("CERRANDO CONTROLADOR ...")
        print("═" * 50 + "\n")

    def publish_final_status(self):
        final_left_length, final_right_length, final_left_angle, final_right_angle = self.calculate_cable_parameters(self.current_x, self.current_y)
        effector_msg = EffectorCoordinates()
        effector_msg.current.x = self.current_x
        effector_msg.current.y = self.current_y
        effector_msg.target.x = self.target_x
        effector_msg.target.y = self.target_y
        self.effector_coords_pub.publish(effector_msg)
        final_log = String()
        final_log.data = f"TRAYECTORIA COMPLETADA: ({self.initial_x:.3f}, {self.initial_y:.3f}) → ({self.current_x:.3f}, {self.current_y:.3f})"
        self.effector_log_pub.publish(final_log)
        self.cable_log_pub.publish(final_log)
        self.pulley_log_pub.publish(final_log)

    def shutdown_node(self):
        print("CERRANDO CONTROLADOR ...")
        self.destroy_node()
        rclpy.shutdown()

    def publish_effector_parameters(self):
        if self.objective_achieved:
            return
        effector_msg = EffectorCoordinates()
        effector_msg.current.x = self.current_x
        effector_msg.current.y = self.current_y
        effector_msg.target.x = self.target_x
        effector_msg.target.y = self.target_y
        self.effector_coords_pub.publish(effector_msg)
        log_msg = String()
        if self.moving:
            progress_percent = self.animation_progress * 100
            log_msg.data = f"EFECTOR: PROGRESO {progress_percent:.1f} % - ACTUAL = ({self.current_x:.3f}, {self.current_y:.3f}) m, OBJETIVO = ({self.target_x:.3f}, {self.target_y:.3f}) m"
        else:
            log_msg.data = f"EFECTOR: EN REPOSO - POSICIÓN = ({self.current_x:.3f}, {self.current_y:.3f}) m"
        self.effector_log_pub.publish(log_msg)

    def publish_cable_parameters(self):
        if self.objective_achieved:
            return
        current_left_length, current_right_length, current_left_angle, current_right_angle = self.calculate_cable_parameters(self.current_x, self.current_y)
        cable_msg = CableParameters()
        cable_msg.left_length = current_left_length
        cable_msg.right_length = current_right_length
        cable_msg.left_angle = current_left_angle
        cable_msg.right_angle = current_right_angle
        self.cable_params_pub.publish(cable_msg)
        log_msg = String()
        log_msg.data = f"CABLES: (L1, L2) = ({current_left_length:.3f} m, {current_right_length:.3f} m), (Q1, Q2) =({current_left_angle:.2f} °, {current_right_angle:.2f} °)"
        self.cable_log_pub.publish(log_msg)

    def publish_pulley_parameters(self):
        if self.objective_achieved:
            return
        if self.moving:
            current_left_cable_change, current_right_cable_change, current_left_pulley_angle, current_right_pulley_angle = self.calculate_pulley_movement(self.initial_x, self.initial_y, self.current_x, self.current_y)
        else:
            current_left_cable_change = 0.0
            current_right_cable_change = 0.0
            current_left_pulley_angle = 0.0
            current_right_pulley_angle = 0.0
        pulley_msg = PulleyParameters()
        pulley_msg.left_cable_change = current_left_cable_change
        pulley_msg.right_cable_change = current_right_cable_change
        pulley_msg.left_pulley_angle = current_left_pulley_angle
        pulley_msg.right_pulley_angle = current_right_pulley_angle
        self.pulley_params_pub.publish(pulley_msg)
        log_msg = String()
        log_msg.data = f"POLEAS: DIFERENCIA = ({current_left_cable_change:+.3f} m, {current_right_cable_change:+.3f} m), ÁNGULO = ({current_left_pulley_angle:+.3f} rad, {current_right_pulley_angle:+.3f} rad)"
        self.pulley_log_pub.publish(log_msg)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_pulley_pos, self.right_pulley_pos]
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    controller = CableRobotController()
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
        except:
            pass

if __name__ == '__main__':
    main()