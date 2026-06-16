from cdpr_interfaces.srv import DrawFigure
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from .figures_library import FiguresLibrary


class FigureService(Node):

    # __init__() function
    # Initializes the node, service, and publisher for managing figures
    def __init__(self):
        super().__init__('figures_service')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(
            Path, '/cdpr', qos_profile)
        self.service_ = self.create_service(
            DrawFigure, '/draw_figure', self.draw_figure_callback)
        self.get_logger().info('FIGURES SERVICE READY')

    # draw_figure_callback() function
    # Generates and publishes the requested figure through the service
    def draw_figure_callback(self, request, response):
        try:
            points = FiguresLibrary.get_figure(
                request.category,
                request.name,
                request.size
            )
            path = FiguresLibrary.build_path(points)
            self.publisher_.publish(path)
            response.success = True
            response.message = 'FIGURE PUBLISHED SUCCESSFULLY'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # __del__() function
    # Safely shuts down ROS2 node
    def __del__(self):
        try:
            self.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


# main() function
# Initializes and keeps the figure service node running
def main(args=None):
    rclpy.init(args=args)
    node = FigureService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCLOSING NODE...')
    except Exception as e:
        print(f'ERROR: {e}')
    finally:
        pass


if __name__ == '__main__':
    main()
