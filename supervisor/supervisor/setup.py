class Setup:
    class Topics:
        demo = "/topic"
        odometry = "/car/odom"
        clock = "/clock"
        events = "/events"
        penalties = "/events/penalties"

        class Fuzzed:
            odometry = "/car/odom/fuzzed"

        class SimulatorManager:
            gazebo_status = "/simulator_manager/gazebo_status"

        class Driver:
            metadata = "/driver/meta"
            stdout = "/driver/stdout"
            stderr = "/driver/stderr"

        class Camera0:
            raw = "/camera0/image_raw"
            labelled = "/camera0/image_labelled"
            fuzzed = "/camera0/image_fuzzed"

        class Camera1:
            raw = "/camera1/image_raw"
            labelled = "/camera1/image_labelled"
            fuzzed = "/camera1/image_fuzzed"

        class Camera2:
            raw = "/camera2/image_raw"
            labelled = "/camera2/image_labelled"
            fuzzed = "/camera2/image_fuzzed"

        class Camera3PV:
            raw = "/camera_3pv/image_raw"

        class Eval:
            velocity = "/eval/velocity"
            acceleration = "/eval/acceleration"
            map = "/eval/map"
            racingline_position = "/eval/racingline_position"
            lap = "/eval/lap"
            report = "/eval/report"
            vehicle_unmoving = "/eval/vehicle_unmoving"

    class Services:

        class SimulatorManager:
            launch_gazebo = "/simulator_manager/launch_gazebo"
            stop_gazebo = "/simulator_manager/stop_gazebo"
            pause_gazebo = "/simulator_manager/pause_gazebo"
            unpause_gazebo = "/simulator_manager/unpause_gazebo"
            reset_gazebo = "/simulator_manager/reset_gazebo"
            get_maps = "/simulator_manager/get_maps"
            get_cars = "/simulator_manager/get_cars"
            spawn_car = "/simulator_manager/spawn_car"
            get_drivers = "/simulator_manager/get_drivers"
            start_driver = "/simulator_manager/start_driver"
            stop_driver = "/simulator_manager/stop_driver"
            restart_driver = "/simulator_manager/restart_driver"
            get_static_map = "/simulator_manager/get_static_map"
            get_optimumlap_map = "/simulator_manager/get_optimumlap_map"
            get_random_map = "/simulator_manager/get_random_map"
            get_fuzzed_map = "/simulator_manager/get_fuzzed_map"
            get_semi_static_map = "/simulator_manager/get_semi_static_map"

        class VehicleEvaluator:
            get_report = "/reports/get"
            get_reports = "/reports/get_all"
            add_report = "/reports/add"
            delete_report = "/reports/delete"

        class Fuzzing:
            get_configuration = "/fuzzing/get_config"
            set_configuration = "/fuzzing/set_config"
            reset = "/fuzzing/reset"
