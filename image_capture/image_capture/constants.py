from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Custom Communication topics
#OUTPUT_TOPIC = "image_capture/output"
#LINE_DIRECTION_TOPIC = "image_capture/direction"

highprofile = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)

lowprofile = QoSProfile(
    depth=3,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


# RAE Available NodesAUDIO_IN_TOPIC = '/audio_in'
BATTERY_STATUS_TOPIC = "/battery_status"
CMD_VEL_TOPIC = "/cmd_vel"
DIAGNOSTICS_TOPIC = "/diagnostics"
DIFF_CONTROLLER_ODOM_TOPIC = "/diff_controller/odom"
DIFF_CONTROLLER_TRANSITION_EVENT_TOPIC = "/diff_controller/transition_event"
DYNAMIC_JOINT_STATES_TOPIC = "/dynamic_joint_states"
IMU_DATA_TOPIC = "/imu/data"
JOINT_STATE_BROADCASTER_TRANSITION_EVENT_TOPIC = (
    "/joint_state_broadcaster/transition_event"
)
JOINT_STATES_TOPIC = "/joint_states"
LCD_TOPIC = "/lcd"
LEDS_TOPIC = "/leds"
ODOMETRY_FILTERED_TOPIC = "/odometry/filtered"
PARAMETER_EVENTS_TOPIC = "/parameter_events"
RAE_IMU_DATA_TOPIC = "/rae/imu/data"
RAE_IMU_MAG_TOPIC = "/rae/imu/mag"
RAE_LEFT_BACK_CAMERA_INFO_TOPIC = "/rae/left_back/camera_info"
RAE_LEFT_BACK_IMAGE_RAW_TOPIC = "/rae/left_back/image_raw"
RAE_LEFT_BACK_IMAGE_RAW_COMPRESSED_TOPIC = "/rae/left_back/image_raw/compressed"
RAE_LEFT_BACK_IMAGE_RAW_COMPRESSED_DEPTH_TOPIC = (
    "/rae/left_back/image_raw/compressedDepth"
)
RAE_LEFT_BACK_IMAGE_RAW_THEORA_TOPIC = "/rae/left_back/image_raw/theora"
RAE_RIGHT_CAMERA_INFO_TOPIC = "/rae/right/camera_info"
RAE_RIGHT_IMAGE_RAW_TOPIC = "/rae/right/image_raw"
RAE_RIGHT_IMAGE_RAW_COMPRESSED_TOPIC = "/rae/right/image_raw/compressed"
RAE_RIGHT_IMAGE_RAW_COMPRESSED_DEPTH_TOPIC = "/rae/right/image_raw/compressedDepth"
RAE_RIGHT_IMAGE_RAW_THEORA_TOPIC = "/rae/right/image_raw/theora"
RAE_STEREO_BACK_CAMERA_INFO_TOPIC = "/rae/stereo_back/camera_info"
RAE_STEREO_BACK_IMAGE_RAW_TOPIC = "/rae/stereo_back/image_raw"
RAE_STEREO_BACK_IMAGE_RAW_COMPRESSED_TOPIC = "/rae/stereo_back/image_raw/compressed"
RAE_STEREO_BACK_IMAGE_RAW_COMPRESSED_DEPTH_TOPIC = (
    "/rae/stereo_back/image_raw/compressedDepth"
)
RAE_STEREO_BACK_IMAGE_RAW_THEORA_TOPIC = "/rae/stereo_back/image_raw/theora"
RAE_STEREO_FRONT_CAMERA_INFO_TOPIC = "/rae/stereo_front/camera_info"
RAE_STEREO_FRONT_IMAGE_RAW_TOPIC = "/rae/stereo_front/image_raw"
RAE_STEREO_FRONT_IMAGE_RAW_COMPRESSED_TOPIC = "/rae/stereo_front/image_raw/compressed"
RAE_STEREO_FRONT_IMAGE_RAW_COMPRESSED_DEPTH_TOPIC = (
    "/rae/stereo_front/image_raw/compressedDepth"
)
RAE_STEREO_FRONT_IMAGE_RAW_THEORA_TOPIC = "/rae/stereo_front/image_raw/theora"
ROBOT_DESCRIPTION_TOPIC = "/robot_description"
ROSOUT_TOPIC = "/rosout"
SET_POSE_TOPIC = "/set_pose"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
