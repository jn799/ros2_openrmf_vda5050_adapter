import rclpy
from vda5050_nav2_bridge.nav2_bridge import Nav2Bridge


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
