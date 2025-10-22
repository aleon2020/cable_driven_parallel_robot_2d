# Implementación de un robot por cables para el control
# de un efector final en diversas tareas.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# COMPILACIÓN, ENLAZADO Y EJECUCIÓN
# TERMINAL 1: colcon build --symlink-install
# TERMINAL 2: ros2 run cdpr_2d version1_controller
# TERMINAL 3: ros2 topic echo /effector_coordinates
# TERMINAL 4: ros2 topic echo /cable_parameters
# TERMINAL 5: ros2 topic echo /pulley_parameters
# TERMINAL 6: ros2 topic pub --once /version1 geometry_msgs/msg/PoseStamped "{pose: {position: {x: 0.30, y: 0.30, z: 0.0}}}"

class Version1Controller(Node):

    def __init__(self):
        super().__init__('version1_controller')

        # PARÁMETROS FÍSICOS DEL SISTEMA
        self.plane_width = 1.0
        self.plane_height = 1.0
        self.effector_width = 0.1
        self.effector_height = 0.1
        self.pulley_radius = 0.05

        # POSICIONES INICIALES
        self.current_x = self.plane_width / 2.0
        self.current_y = self.plane_height / 2.0
        self.reference_x = self.plane_width / 2.0
        self.reference_y = self.plane_height / 2.0

        # CONFIGURACIÓN DE LA CALIDAD DE SERVICIO (QOS)
        qos_profile = QoSProfile(depth=10)

        # CREACIÓN DE PUBLICADORES
        self.effector_coordinates_publisher = self.create_publisher(Point, '/effector_coordinates', qos_profile)
        self.cable_parameters_publisher = self.create_publisher(Float32MultiArray, '/cable_parameters', qos_profile)
        self.pulley_parameters_publisher = self.create_publisher(Float32MultiArray, '/pulley_parameters', qos_profile)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)

        # SUBSCRIPCIÓN AL TOPIC /version1 (PoseStamped)
        self.coordinates_subscriber = self.create_subscription(PoseStamped, '/version1', self.position_callback, qos_profile)
        
        # CÁLCULO DE PARÁMETROS DE REFERENCIA INICIALES
        (self.reference_left_cable_length, self.reference_right_cable_length,
         self.reference_left_cable_angle, self.reference_right_cable_angle) = self.calculate_cable_parameters(self.reference_x, self.reference_y)
        
        self.received = False
        self.get_logger().info("CONTROLADOR ACTIVADO. ESPERANDO COORDENADAS ...")

    # --------------------------
    # CÁLCULO DE PARÁMETROS
    # --------------------------
    def calculate_cable_parameters(self, x, y):
        # COORDENADAS DE LAS POLEAS
        left_pulley_x, left_pulley_y = 0.0, self.plane_height
        right_pulley_x, right_pulley_y = self.plane_width, self.plane_height

        # PUNTOS DE CONEXIÓN
        left_connection_x = x - self.effector_width / 2.0
        left_connection_y = y + self.effector_height / 2.0
        right_connection_x = x + self.effector_width / 2.0
        right_connection_y = y + self.effector_height / 2.0

        # LONGITUDES DE LOS CABLES
        left_cable_length = math.sqrt((left_connection_x - left_pulley_x)**2 + (left_connection_y - left_pulley_y)**2)
        right_cable_length = math.sqrt((right_connection_x - right_pulley_x)**2 + (right_connection_y - right_pulley_y)**2)

        # ÁNGULOS DE LOS CABLES
        left_cable_angle = math.degrees(math.atan2(left_connection_x - left_pulley_x, left_pulley_y - left_connection_y))
        right_cable_angle = math.degrees(math.atan2(right_pulley_x - right_connection_x, right_pulley_y - right_connection_y))

        return left_cable_length, right_cable_length, left_cable_angle, right_cable_angle

    def calculate_pulley_changes(self, x, y):
        l1, l2, _, _ = self.calculate_cable_parameters(x, y)
        delta_l1 = l1 - self.reference_left_cable_length
        delta_l2 = l2 - self.reference_right_cable_length
        pulley1_angle = delta_l1 / self.pulley_radius
        pulley2_angle = delta_l2 / self.pulley_radius
        return delta_l1, delta_l2, pulley1_angle, pulley2_angle

    # --------------------------
    # CALLBACK
    # --------------------------
    def position_callback(self, msg: PoseStamped):
        if self.received:
            return
        try:
            pos = msg.pose.position
        except AttributeError:
            self.get_logger().error("ERROR. FORMATO CORRECTO DEL MENSAJE: {pose: {position: {x:..., y:..., z:...}}}")
            return

        self.received = True
        self.current_x, self.current_y = float(pos.x), float(pos.y)

        # CÁLCULOS
        delta_l1, delta_l2, pulley1_angle, pulley2_angle = self.calculate_pulley_changes(self.current_x, self.current_y)
        left_len, right_len, left_ang, right_ang = self.calculate_cable_parameters(self.current_x, self.current_y)

        # PUBLICACIÓN DE LA POSICIÓN DEL EFECTOR
        effector_msg = Point()
        effector_msg.x = self.current_x
        effector_msg.y = self.current_y
        effector_msg.z = 0.0
        self.effector_coordinates_publisher.publish(effector_msg)

        # PUBLICACIÓN DE LOS PARÁMETROS DE LOS CABLES
        cable_msg = Float32MultiArray()
        cable_msg.data = [left_len, right_len, left_ang, right_ang]
        self.cable_parameters_publisher.publish(cable_msg)

        # PUBLICACIÓN DE LOS PARÁMETROS DE LAS POLEAS
        pulley_msg = Float32MultiArray()
        pulley_msg.data = [delta_l1, delta_l2, pulley1_angle, pulley2_angle]
        self.pulley_parameters_publisher.publish(pulley_msg)

        # PUBLICACIÓN DE JOINT STATES
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [pulley1_angle, pulley2_angle]
        self.joint_state_publisher.publish(joint_state_msg)

        # VISUALIZACIÓN
        print("=" * 100)
        print("POSICIÓN DEL EFECTOR FINAL (X, Y)")
        print(f"• POSICIÓN OBJETIVO: ({self.current_x:.3f}, {self.current_y:.3f}) m")
        print(f"LONGITUDES DE CADA CABLE (L1, L2)")
        print(f"• L1: {left_len:.3f} m, L2: {right_len:.3f} m")
        print("ÁNGULOS DE CADA CABLE (Q1, Q2)")
        print(f"• Q1: {left_ang:.2f} °, Q2: {right_ang:.2f} °")
        print("MOVIMIENTO DE CADA POLEA (P1, P2)")
        print(f"• CABLE ELONGADO / RECOGIDO:  P1: {delta_l1:+.3f} m,   P2: {delta_l2:+.3f} m")
        print(f"• ÁNGULOS DE GIRO (RADIANES): P1: {pulley1_angle:+.3f} rad, P2: {pulley2_angle:+.3f} rad")
        print("=" * 100)
        self.get_logger().info("CERRANDO CONTROLADOR ...")

        self.create_timer(3.0, self.shutdown_node)

    # --------------------------
    # CIERRE DEL NODO
    # --------------------------
    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = Version1Controller()
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
