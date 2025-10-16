import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import time
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from interfaces_package.msg import EffectorPosition, CableParameters, PulleyParameters


class Version1Controller(Node):

    def __init__(self):
        super().__init__('version1_controller')
        self.platform_width = 1.0
        self.platform_height = 1.0
        self.effector_width = 0.1
        self.effector_height = 0.1
        self.pulley_radius = 0.05
        self.current_x = self.platform_width / 2.0
        self.current_y = self.platform_height / 2.0
        self.reference_x = self.platform_width / 2.0
        self.reference_y = self.platform_height / 2.0
        qos_profile = QoSProfile(depth=10)
        self.effector_coords_pub = self.create_publisher(EffectorPosition, '/effector_coordinates', qos_profile)
        self.cable_params_pub = self.create_publisher(CableParameters, '/cable_parameters', qos_profile)
        self.pulley_params_pub = self.create_publisher(PulleyParameters, '/pulley_parameters', qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.coords_sub = self.create_subscription(EffectorPosition, '/version1', self.position_callback, qos_profile)
        (self.ref_left_length, self.ref_right_length, self.ref_left_angle, self.ref_right_angle) = self.calculate_cable_parameters(self.reference_x, self.reference_y)
        self.received = False
        self.get_logger().info("VERSION1 CONTROLADOR ACTIVADO. Esperando posición en /version1 ...")

    def calculate_cable_parameters(self, x, y):
        left_pulley_x, left_pulley_y = 0.0, self.platform_height
        right_pulley_x, right_pulley_y = self.platform_width, self.platform_height
        left_conn_x = x - self.effector_width / 2.0
        left_conn_y = y + self.effector_height / 2.0
        right_conn_x = x + self.effector_width / 2.0
        right_conn_y = y + self.effector_height / 2.0
        left_len = math.sqrt((left_conn_x - left_pulley_x)**2 + (left_conn_y - left_pulley_y)**2)
        right_len = math.sqrt((right_conn_x - right_pulley_x)**2 + (right_conn_y - right_pulley_y)**2)
        left_ang = math.degrees(math.atan2(left_conn_x - left_pulley_x, left_pulley_y - left_conn_y))
        right_ang = math.degrees(math.atan2(right_pulley_x - right_conn_x, right_pulley_y - right_conn_y))
        return left_len, right_len, left_ang, right_ang

    def calculate_pulley_changes(self, x, y):
        l1, l2, _, _ = self.calculate_cable_parameters(x, y)
        delta_l1 = l1 - self.ref_left_length
        delta_l2 = l2 - self.ref_right_length
        pulley1_angle = delta_l1 / self.pulley_radius
        pulley2_angle = delta_l2 / self.pulley_radius
        return delta_l1, delta_l2, pulley1_angle, pulley2_angle

    def position_callback(self, msg):
        if self.received:
            return
        try:
            pos: Point = msg.position
        except AttributeError:
            self.get_logger().error("Mensaje inválido en /version1. Se esperaba {position: {x:..., y:...}}")
            return
        self.received = True
        self.current_x, self.current_y = float(pos.x), float(pos.y)
        delta_l1, delta_l2, pulley1_angle, pulley2_angle = self.calculate_pulley_changes(self.current_x, self.current_y)
        left_len, right_len, left_ang, right_ang = self.calculate_cable_parameters(self.current_x, self.current_y)
        effector_msg = EffectorPosition()
        effector_msg.position.x = self.current_x
        effector_msg.position.y = self.current_y
        self.effector_coords_pub.publish(effector_msg)
        cable_msg = CableParameters()
        cable_msg.left_length = left_len
        cable_msg.right_length = right_len
        cable_msg.left_angle = left_ang
        cable_msg.right_angle = right_ang
        self.cable_params_pub.publish(cable_msg)
        pulley_msg = PulleyParameters()
        pulley_msg.left_cable_change = delta_l1
        pulley_msg.right_cable_change = delta_l2
        pulley_msg.left_pulley_angle = pulley1_angle
        pulley_msg.right_pulley_angle = pulley2_angle
        self.pulley_params_pub.publish(pulley_msg)
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [pulley1_angle, pulley2_angle]
        self.joint_pub.publish(joint_state)
        print("\n================= RESULTADOS =================")
        print(f"POSICIÓN EFECTOR: ({self.current_x:.3f}, {self.current_y:.3f}) m")
        print(f"CABLES:")
        print(f"  Longitudes -> Izq: {left_len:.3f} m, Der: {right_len:.3f} m")
        print(f"  Ángulos    -> Izq: {left_ang:.2f}°, Der: {right_ang:.2f}°")
        print(f"POLEAS:")
        print(f"  ΔLongitudes -> Izq: {delta_l1:+.3f} m, Der: {delta_l2:+.3f} m")
        print(f"  Ángulos rad -> Izq: {pulley1_angle:+.3f} rad, Der: {pulley2_angle:+.3f} rad")
        print("==============================================\n")
        self.get_logger().info("Datos publicados. El nodo se cerrará en 2 segundos...")
        time.sleep(2.0)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = Version1Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()