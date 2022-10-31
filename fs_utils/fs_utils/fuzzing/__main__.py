import rclpy
from . import Fuzzing


def main(args=None):
    rclpy.init(args=args)

    fuzzer = Fuzzing()

    rclpy.spin(fuzzer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fuzzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
