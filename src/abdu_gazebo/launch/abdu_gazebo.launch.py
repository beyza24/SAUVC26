"""
ABDÜ su altı dünyasını Gazebo Classic ile başlatır ve ROV'u spawn eder.

Kullanim:
  ros2 launch abdu_gazebo abdu_gazebo.launch.py
  ros2 launch abdu_gazebo abdu_gazebo.launch.py rviz:=true
  ros2 launch abdu_gazebo abdu_gazebo.launch.py otonom:=false
"""

import os
import subprocess
from pathlib import Path
from tempfile import gettempdir

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _patched_world(src_world: str, scripts: str) -> str:
    text = Path(src_world).read_text(encoding="utf-8")
    text = text.replace("__ABDU_WATER_SCRIPTS__", scripts)
    out_dir = Path(gettempdir()) / "abdu_gazebo"
    out_dir.mkdir(parents=True, exist_ok=True)
    dest = out_dir / "abdu_sualti.world"
    dest.write_text(text, encoding="utf-8")
    return str(dest)


def _prepend_env(name: str, value: str) -> str:
    current = os.environ.get(name, "")
    parts = [p for p in current.split(":") if p]
    if value and value not in parts:
        parts.insert(0, value)
    joined = ":".join(parts)
    os.environ[name] = joined
    return joined


def _seed_gazebo_paths(models_path: str, plugin_path: str):
    system_share = "/usr/share/gazebo-11"
    system_plugin = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
    ogre_path = "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0"
    _prepend_env("GAZEBO_MODEL_PATH", os.path.join(system_share, "models"))
    _prepend_env("GAZEBO_MODEL_PATH", models_path)
    _prepend_env("GAZEBO_PLUGIN_PATH", system_plugin)
    _prepend_env("GAZEBO_PLUGIN_PATH", plugin_path)
    _prepend_env("GAZEBO_RESOURCE_PATH", system_share)
    _prepend_env("GAZEBO_RESOURCE_PATH", models_path)
    if os.path.isdir(ogre_path):
        _prepend_env("OGRE_RESOURCE_PATH", ogre_path)
    return (
        os.environ["GAZEBO_MODEL_PATH"],
        os.environ["GAZEBO_PLUGIN_PATH"],
        os.environ["GAZEBO_RESOURCE_PATH"],
    )


def generate_launch_description():
    gazebo_share = get_package_share_directory("abdu_gazebo")
    desc_share = get_package_share_directory("abdu_description")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    gazebo_prefix = get_package_prefix("abdu_gazebo")

    models_path = os.path.join(gazebo_share, "models")
    plugin_path = os.path.join(gazebo_prefix, "lib")
    water_scripts = os.path.join(models_path, "abdu_water", "materials", "scripts")
    world_path = _patched_world(
        os.path.join(gazebo_share, "worlds", "abdu_sualti.world"),
        water_scripts,
    )
    xacro_path = os.path.join(desc_share, "urdf", "abdu_rov.urdf.xacro")
    rviz_path = os.path.join(desc_share, "rviz", "abdu.rviz")
    urdf_dir = Path(gettempdir()) / "abdu_gazebo"
    urdf_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = urdf_dir / "abdu_rov.urdf"
    urdf_xml = subprocess.check_output(["xacro", xacro_path], text=True)
    urdf_path.write_text(urdf_xml, encoding="utf-8")

    os.environ["GAZEBO_IP"] = "127.0.0.1"
    os.environ["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11346"
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    model_path, plugin_env, resource_path = _seed_gazebo_paths(models_path, plugin_path)
    os.environ["GAZEBO_MODEL_PATH"] = model_path
    os.environ["GAZEBO_RESOURCE_PATH"] = resource_path
    set_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    set_plugin_path = SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", plugin_env)
    set_resource_path = SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", resource_path)
    set_gazebo_ip = SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1")
    set_master_uri = SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11346")
    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", os.environ.get("ROS_DOMAIN_ID", "42"))

    declare_gui = DeclareLaunchArgument("gui", default_value="true")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false")
    declare_otonom = DeclareLaunchArgument(
        "otonom",
        default_value="true",
        description="Denetim görevini (boru, enkaz, kapı) çalıştır",
    )
    declare_preview = DeclareLaunchArgument(
        "preview",
        default_value="true",
        description="Tespit kamera penceresini ac (X veya q ile kapanir)",
    )
    declare_x = DeclareLaunchArgument("spawn_x", default_value="-3.6")
    declare_y = DeclareLaunchArgument("spawn_y", default_value="0.0")
    declare_z = DeclareLaunchArgument("spawn_z", default_value="-1.05")
    declare_yaw = DeclareLaunchArgument("spawn_yaw", default_value="0.0")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "gui": LaunchConfiguration("gui"),
            "verbose": "false",
        }.items(),
    )

    robot_description = ParameterValue(urdf_xml, value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="abdu_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", str(urdf_path),
            "-entity", "abdu_rov",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
    )

    inspect_mission = Node(
        package="abdu_control",
        executable="inspect_mission",
        output="screen",
        condition=IfCondition(LaunchConfiguration("otonom")),
        parameters=[
            {
                "use_sim_time": True,
                "preview": ParameterValue(LaunchConfiguration("preview"), value_type=bool),
            }
        ],
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        set_resource_path,
        set_gazebo_ip,
        set_master_uri,
        set_domain,
        declare_gui,
        declare_rviz,
        declare_otonom,
        declare_preview,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=9.0, actions=[inspect_mission]),
        rviz,
    ])
