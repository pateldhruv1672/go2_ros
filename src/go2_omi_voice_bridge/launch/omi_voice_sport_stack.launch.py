from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def generate_launch_description():
    package_share = FindPackageShare("go2_omi_voice_bridge")
    config_file = PathJoinSubstitution([package_share, "config", "omi_voice.yaml"])

    adapter_mode = LaunchConfiguration("adapter_mode")
    ble_device_name = LaunchConfiguration("ble_device_name")
    ble_device_address = LaunchConfiguration("ble_device_address")

    omi_connect_preflight = LaunchConfiguration("omi_connect_preflight")
    omi_connect_required = LaunchConfiguration("omi_connect_required")
    omi_scan_timeout_sec = LaunchConfiguration("omi_scan_timeout_sec")
    omi_connect_timeout_sec = LaunchConfiguration("omi_connect_timeout_sec")

    require_wake_word = LaunchConfiguration("require_wake_word")
    require_confirmation_for_skills = LaunchConfiguration("require_confirmation_for_skills")

    tts_enabled = LaunchConfiguration("tts_enabled")
    local_speaker_enabled = LaunchConfiguration("local_speaker_enabled")
    local_speaker_backend = LaunchConfiguration("local_speaker_backend")

    network_interface = LaunchConfiguration("network_interface")
    motion_mode = LaunchConfiguration("motion_mode")
    allow_during_navigation = LaunchConfiguration("allow_during_navigation")
    allow_medium_risk = LaunchConfiguration("allow_medium_risk")
    allow_high_risk = LaunchConfiguration("allow_high_risk")
    allow_parameter_commands = LaunchConfiguration("allow_parameter_commands")

    def guarded_nodes(context, *args, **kwargs):
        mode = adapter_mode.perform(context)
        address = ble_device_address.perform(context)
        name = ble_device_name.perform(context)

        if mode == "ble_audio" and _as_bool(omi_connect_preflight.perform(context)):
            from go2_omi_voice_bridge.omi_ble_preflight import run_omi_ble_preflight

            ok = run_omi_ble_preflight(
                address=address,
                name=name,
                scan_timeout_sec=float(omi_scan_timeout_sec.perform(context)),
                connect_timeout_sec=float(omi_connect_timeout_sec.perform(context)),
            )

            if not ok and _as_bool(omi_connect_required.perform(context)):
                raise RuntimeError(
                    "Omi BLE preflight failed. Force-stopping launch before starting voice/motion nodes. "
                    f"address={address} name={name}"
                )

        return [
            Node(
                package="go2_omi_voice_bridge",
                executable="omi_ble_bridge_node",
                name="go2_omi_bridge",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "adapter_mode": adapter_mode,
                        "ble_device_name": ble_device_name,
                        "ble_device_address": ble_device_address,
                    },
                ],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="stt_node",
                name="go2_voice_stt_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "adapter_mode": adapter_mode,
                        "ble_device_name": ble_device_name,
                        "ble_device_address": ble_device_address,
                    },
                ],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="tts_node",
                name="go2_tts_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "tts_enabled": tts_enabled,
                        "local_speaker_enabled": local_speaker_enabled,
                        "local_speaker_backend": local_speaker_backend,
                        "interrupt_previous_by_default": True,
                        "speak_vlm_status": False,
                        "max_spoken_sentences": 1,
                        "local_speaker_rate": 145,
                        "local_speaker_volume": 70,
                    },
                ],
            ),
            Node(
                package="go2_omi_voice_bridge",
                executable="omi_skill_intent_router",
                name="omi_skill_intent_router",
                output="screen",
                parameters=[
                    {
                        "transcript_topic": "/go2_voice/transcript",
                        "motion_skill_topic": "/motion_skills/command",
                        "semantic_command_topic": "/semantic_nav/command",
                        "tts_topic": "/go2_tts/say",
                        "status_topic": "/go2_voice/skill_router_status",
                        "processed_stt_topic": "/go2_voice/processed_stt",
                        "require_wake_word": require_wake_word,
                        "require_confirmation_for_skills": require_confirmation_for_skills,
                        "print_processed_stt": True,
                    }
                ],
            ),
            Node(
                package="go2_agentic_motion_skills",
                executable="webrtc_motion_skill_agent_node",
                name="webrtc_motion_skill_agent_node",
                output="screen",
                parameters=[
                    {
                        "command_topic": "/motion_skills/command",
                        "webrtc_req_topic": "/webrtc_req",
                        "sport_topic": "rt/api/sport/request",
                        "allow_medium_risk": allow_medium_risk,
                        "allow_high_risk": allow_high_risk,
                        "allow_parameter_commands": allow_parameter_commands,
                    }
                ],
            ),
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="ble_audio"),
            DeclareLaunchArgument("ble_device_name", default_value="Omi DevK"),
            DeclareLaunchArgument("ble_device_address", default_value="EF:1C:34:C6:25:92"),

            DeclareLaunchArgument("omi_connect_preflight", default_value="true"),
            DeclareLaunchArgument("omi_connect_required", default_value="true"),
            DeclareLaunchArgument("omi_scan_timeout_sec", default_value="12.0"),
            DeclareLaunchArgument("omi_connect_timeout_sec", default_value="10.0"),

            DeclareLaunchArgument("tts_enabled", default_value="true"),
            DeclareLaunchArgument("local_speaker_enabled", default_value="true"),
            DeclareLaunchArgument("local_speaker_backend", default_value="auto"),

            DeclareLaunchArgument("require_wake_word", default_value="true"),
            DeclareLaunchArgument("require_confirmation_for_skills", default_value="true"),

            DeclareLaunchArgument("network_interface", default_value=""),
            DeclareLaunchArgument("motion_mode", default_value="normal"),
            DeclareLaunchArgument("allow_during_navigation", default_value="false"),
            DeclareLaunchArgument("allow_medium_risk", default_value="true"),
            DeclareLaunchArgument("allow_high_risk", default_value="false"),
            DeclareLaunchArgument("allow_parameter_commands", default_value="false"),

            OpaqueFunction(function=guarded_nodes),
        ]
    )
