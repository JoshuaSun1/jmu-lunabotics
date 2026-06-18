import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on", "enabled"}


def _camera_node(context):
    driver = LaunchConfiguration("webcam_driver").perform(context)
    video_device = LaunchConfiguration("video_device").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    camera_info_file = LaunchConfiguration("camera_info_file").perform(context).strip()

    parameters = {"camera_name": camera_name}
    if camera_info_file:
        if not os.path.isfile(camera_info_file):
            raise RuntimeError(f"Camera calibration file was not found: {camera_info_file}")
        parameters["camera_info_url"] = f"file://{camera_info_file}"

    if driver == "v4l2_camera":
        parameters["video_device"] = video_device
        parameters["camera_frame_id"] = frame_id
        return Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            output="screen",
            parameters=[parameters],
            remappings=[
                ("image_raw", image_topic),
                ("camera_info", camera_info_topic),
            ],
        )

    if driver == "usb_cam":
        parameters["video_device"] = video_device
        parameters["frame_id"] = frame_id
        return Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="camera",
            output="screen",
            parameters=[parameters],
            remappings=[
                ("image_raw", image_topic),
                ("camera_info", camera_info_topic),
            ],
        )

    raise RuntimeError(
        f"Unsupported webcam_driver={driver}. Use 'v4l2_camera' or 'usb_cam'."
    )


def _launch_setup(context, *args, **kwargs):
    actions = []

    if _as_bool(LaunchConfiguration("enable_camera").perform(context)):
        actions.append(_camera_node(context))
    else:
        actions.append(LogInfo(msg="Camera launch disabled."))

    if _as_bool(LaunchConfiguration("enable_apriltag").perform(context)):
        apriltag_ns = LaunchConfiguration("apriltag_ns").perform(context)
        apriltag_tag_id = int(LaunchConfiguration("apriltag_tag_id").perform(context))
        apriltag_tag_family = LaunchConfiguration("apriltag_tag_family").perform(context)
        apriltag_tag_size = float(
            LaunchConfiguration("apriltag_tag_size_meters").perform(context)
        )
        rectified_image_topic = LaunchConfiguration("rectified_image_topic").perform(context)
        image_topic = LaunchConfiguration("image_topic").perform(context)
        camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
        frame_id = LaunchConfiguration("frame_id").perform(context)
        apriltag_pose_topic = LaunchConfiguration("apriltag_pose_topic").perform(context)
        apriltag_distance_topic = LaunchConfiguration("apriltag_distance_topic").perform(
            context
        )
        apriltag_pose_estimation_method = LaunchConfiguration(
            "apriltag_pose_estimation_method"
        ).perform(context)
        apriltag_threads = int(LaunchConfiguration("apriltag_threads").perform(context))
        apriltag_decimate = float(LaunchConfiguration("apriltag_decimate").perform(context))
        apriltag_blur = float(LaunchConfiguration("apriltag_blur").perform(context))
        apriltag_refine = _as_bool(LaunchConfiguration("apriltag_refine").perform(context))
        apriltag_sharpening = float(
            LaunchConfiguration("apriltag_sharpening").perform(context)
        )
        apriltag_debug = _as_bool(LaunchConfiguration("apriltag_debug").perform(context))
        apriltag_max_hamming = int(
            LaunchConfiguration("apriltag_max_hamming").perform(context)
        )
        tag_frame = f"apriltag_{apriltag_tag_id}"

        actions.extend(
            [
                Node(
                    package="image_proc",
                    executable="rectify_node",
                    name="rectify",
                    output="screen",
                    remappings=[
                        ("image", image_topic),
                        ("camera_info", camera_info_topic),
                        ("image_rect", rectified_image_topic),
                    ],
                ),
                Node(
                    package="apriltag_ros",
                    executable="apriltag_node",
                    namespace=apriltag_ns,
                    name="detector",
                    output="screen",
                    remappings=[
                        ("image_rect", rectified_image_topic),
                        ("camera_info", camera_info_topic),
                        ("detections", f"{apriltag_ns}/detections"),
                    ],
                    parameters=[
                        {
                            "image_transport": "raw",
                            "family": apriltag_tag_family,
                            "size": apriltag_tag_size,
                            "max_hamming": apriltag_max_hamming,
                            "pose_estimation_method": apriltag_pose_estimation_method,
                            "detector": {
                                "threads": apriltag_threads,
                                "decimate": apriltag_decimate,
                                "blur": apriltag_blur,
                                "refine": apriltag_refine,
                                "sharpening": apriltag_sharpening,
                                "debug": apriltag_debug,
                            },
                            "tag": {
                                "ids": [apriltag_tag_id],
                                "frames": [tag_frame],
                                "sizes": [apriltag_tag_size],
                            },
                        }
                    ],
                ),
                Node(
                    package="jmu_lunabotics",
                    executable="publish_apriltag_pose.py",
                    name="apriltag_pose_publisher",
                    output="screen",
                    parameters=[
                        {
                            "tag_frame": tag_frame,
                            "camera_frame": frame_id,
                            "pose_topic": apriltag_pose_topic,
                            "distance_topic": apriltag_distance_topic,
                        }
                    ],
                ),
            ]
        )
    else:
        actions.append(LogInfo(msg="AprilTag launch disabled."))

    if _as_bool(LaunchConfiguration("enable_view_image_raw").perform(context)):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rqt_image_view",
                    "rqt_image_view",
                    LaunchConfiguration("image_topic"),
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
