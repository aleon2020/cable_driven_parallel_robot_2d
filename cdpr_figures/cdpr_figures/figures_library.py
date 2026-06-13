from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class FiguresLibrary:

    FIGURES = {

        # -------
        # LETTERS
        # -------

        'letter': {

            'C': {
                'small': [],
                'medium': [],
                'large': []
            },

            'L': {
                'small': [],
                'medium': [],
                'large': []
            },

            'M': {
                'small': [],
                'medium': [],
                'large': []
            },

            'N': {
                'small': [],
                'medium': [],
                'large': []
            },

            'P': {
                'small': [],
                'medium': [],
                'large': []
            },

            'U': {
                'small': [],
                'medium': [],
                'large': []
            },

            'V': {
                'small': [],
                'medium': [],
                'large': []
            },

            'W': {
                'small': [],
                'medium': [],
                'large': []
            },

            'Z': {
                'small': [],
                'medium': [],
                'large': []
            }

        },

        # -------
        # NUMBERS
        # -------

        'number': {

            '1': {
                'small': [],
                'medium': [],
                'large': []
            },

            '2': {
                'small': [],
                'medium': [],
                'large': []
            },

            '5': {
                'small': [],
                'medium': [],
                'large': []
            },

            '6': {
                'small': [],
                'medium': [],
                'large': []
            },

            '9': {
                'small': [],
                'medium': [],
                'large': []
            }

        },

        # --------
        # POLYGONS
        # --------

        'polygon': {

            'hexagon': {
                'small': [],
                'medium': [],
                'large': []
            },

            'octagon': {
                'small': [],
                'medium': [],
                'large': []
            },

            'rhombus': {
                'small': [],
                'medium': [],
                'large': []
            },

            'square': {
                'small': [(0.3, 0.3), (0.3, 0.7), (0.7, 0.7), (0.7, 0.3), (0.3, 0.3)],
                'medium': [(0.2, 0.2), (0.2, 0.8), (0.8, 0.8), (0.8, 0.2), (0.2, 0.2)],
                'large': [(0.1, 0.1), (0.1, 0.9), (0.9, 0.9), (0.9, 0.1), (0.1, 0.1)]
            },

            'trapezio': {
                'small': [],
                'medium': [],
                'large': []
            },

            'triangle': {
                'small': [(0.3, 0.3), (0.5, 0.7), (0.7, 0.3), (0.3, 0.3)],
                'medium': [(0.2, 0.2), (0.5, 0.8), (0.8, 0.2), (0.2, 0.2)],
                'large': [(0.1, 0.1), (0.5, 0.9), (0.9, 0.1), (0.1, 0.1)]
            }

        },

        # ------
        # SHAPES
        # ------
        'shape': {

            'horizontal_hg': {
                'small': [],
                'medium': [],
                'large': []
            },

            'house': {
                'small': [],
                'medium': [],
                'large': []
            },

            'lightning': {
                'small': [],
                'medium': [],
                'large': []
            },

            'shield': {
                'small': [],
                'medium': [],
                'large': []
            },

            'vertical_hg': {
                'small': [],
                'medium': [],
                'large': []
            }
        }
    }

    @staticmethod
    def get_figure(category, name, size):
        try:
            return FiguresLibrary.FIGURES[category][name][size]
        except KeyError:
            raise ValueError(
                f'FIGURE NOT FOUND: '
                f'category={category}, '
                f'name={name}, '
                f'size={size}'
            )

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
