import os
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package="openpose_ros",
            node_executable="openpose_ros_node",
            output="screen",
            parameters=[os.path.join(os.path.dirname(__file__), "..", "config", "config.yaml")]
        )
    ])
    return ld


if __name__ == "__main__":
    generate_launch_description()
