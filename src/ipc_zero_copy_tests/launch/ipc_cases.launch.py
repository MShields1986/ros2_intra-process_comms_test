"""Run the zero-copy intra-process cases from the blog post.

Launch arguments (all overridable from the CLI):
    message_type                 e.g. "sensor_msgs/PointCloud2" ("list" prints every supported type)
    payload_bytes                target size of the variable-length field (ignored by fixed-size types)
    cases                        "all", a comma list of case ids ("11,12,18"), or "custom"
    custom_publish               custom case: unique_ptr | const_ref
    custom_subscriptions         custom case: comma list of
                                 unique_ptr | shared_ptr | const_shared_ptr | const_shared_ptr_ref | const_ref
    custom_external_subscribers  custom case: number of out-of-process subscribers
    iterations                   measured frames per case
    warmup_iterations            unmeasured frames per case before measuring
    use_intra_process_comms      enable rclcpp intra-process comms (expectations assume true)
    qos_depth                    KeepLast depth (reliable, volatile)
    settle_ms                    after a frame reaches every subscription, keep spinning until
                                 nothing new arrives for this long (catches duplicate deliveries)
    output_dir                   directory (inside container) to write the CSV into
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_ARGS = [
    # name, default, type
    ("message_type", "sensor_msgs/PointCloud2", str),
    ("payload_bytes", "1840000", int),
    ("cases", "all", str),
    ("custom_publish", "unique_ptr", str),
    ("custom_subscriptions", "unique_ptr", str),
    ("custom_external_subscribers", "0", int),
    ("iterations", "100", int),
    ("warmup_iterations", "10", int),
    ("use_intra_process_comms", "true", bool),
    ("qos_depth", "10", int),
    ("settle_ms", "2.0", float),
    ("output_dir", "/data/results", str),
]


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default) for name, default, _ in _ARGS]
    params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type in _ARGS
    }

    runner = Node(
        package="ipc_zero_copy_tests",
        executable="ipc_case_runner",
        # No `name=`: it would remap every node in the process, including
        # the per-case nodes, to the same name.
        output="screen",
        emulate_tty=True,
        parameters=[params],
        on_exit=Shutdown(),
    )

    return LaunchDescription(declared + [runner])
