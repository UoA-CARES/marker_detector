from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace"
    )

    declare_marker_type = DeclareLaunchArgument(
        "marker_type",
        default_value="Aruco",
        description="Type of marker to detect (Aruco or STag)",
    )

    declare_dict_id = DeclareLaunchArgument(
        "dict_id",
        default_value="0",
        description="Marker dictionary ID (e.g. 0 = DICT_4X4_50)",
    )

    declare_marker_size = DeclareLaunchArgument(
        "marker_size",
        default_value="0.1",
        description="Marker size in meters",
    )

    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="Name of the camera topic (default: 'camera')",
    )

    namespace = LaunchConfiguration(declare_namespace.name)
    marker_type = LaunchConfiguration(declare_marker_type.name)
    dict_id = LaunchConfiguration(declare_dict_id.name)
    marker_size = LaunchConfiguration(declare_marker_size.name)
    camera_name = LaunchConfiguration(declare_camera_name.name)

    return LaunchDescription(
        [
            declare_namespace,
            declare_marker_type,
            declare_dict_id,
            declare_marker_size,
            declare_camera_name,
            Node(
                package="marker_detector",
                executable="marker_detector_node",
                name="marker_detector",
                output="screen",
                namespace=namespace,
                parameters=[
                    {
                        declare_marker_type.name: marker_type,
                        declare_dict_id.name: dict_id,
                        declare_marker_size.name: marker_size,
                        declare_camera_name.name: camera_name,
                    }
                ],
            ),
        ]
    )
