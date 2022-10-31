import rclpy
from . import VehicleEvaluatorNode


def main(args=None):
    rclpy.init(args=args)

    node = VehicleEvaluatorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
