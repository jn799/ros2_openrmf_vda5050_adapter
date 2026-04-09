import sys
import rclpy
from .mock_robot import MockRobot


def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
