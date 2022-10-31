import launch
from launch.actions import OpaqueFunction


def with_config(factory):
    # Based on:
    # noqa: E501; REF: https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21

    def generator(context, *args, **kwargs):
        config_path = launch.substitutions.LaunchConfiguration('config_path').perform(context)
        return factory(config_path)

    return [
        launch.actions.DeclareLaunchArgument('config_path', default_value=""),
        OpaqueFunction(function=generator),
    ]
