import rclpy
from .simulator_manager_service import SimulatorManagerService


def main(args=None):
    rclpy.init(args=args)

    minimal_service = SimulatorManagerService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
