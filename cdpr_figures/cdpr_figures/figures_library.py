from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class FiguresLibrary:

    @staticmethod
    def get_figure(category, name, size):

        figures = {
            'polygon': {
                'square': {
                    'small': [
                        (0.3, 0.3),
                        (0.3, 0.7),
                        (0.7, 0.7),
                        (0.7, 0.3),
                        (0.3, 0.3)
                    ],
                    'medium': [
                        (0.2, 0.2),
                        (0.2, 0.8),
                        (0.8, 0.8),
                        (0.8, 0.2),
                        (0.2, 0.2)
                    ],
                    'large': [
                        (0.1, 0.1),
                        (0.1, 0.9),
                        (0.9, 0.9),
                        (0.9, 0.1),
                        (0.1, 0.1)
                    ]
                }
            }
        }
        return figures[category][name][size]

    @staticmethod
    def build_path(points):
        path = Path()
        path.header.frame_id = 'world'
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path.poses.append(pose)
        return path
