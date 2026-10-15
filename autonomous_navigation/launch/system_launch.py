import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. UBXドライバ
        Node(
            package='gnss_ros_standardization',
            executable='ubx_driver_node',
            parameters=['config/ubx_driver.yaml']
        ),
        # 2. RTCMデコーダ
        Node(
            package='gnss_ros_standardization',
            executable='rtcm_decoder_node',
            parameters=[{
                'stream_path': 'ntrip://gnss1:TaroLab@2026@citgnss.taroz.net:2101/RTCM3',
                'observation_topic': '/base/gnss/observation',
                'ephemeris_topic': '/base/gnss/ephemeris'
            }]
        ),
        # 3. RTKノード
        # 3. RTKノード (修正版)
        Node(
            package='gnss_ros_standardization',
            executable='real_time_kinematic',
            parameters=[os.path.join(get_package_share_directory('gnss_ros_standardization'), 'config', 'real_time_kinematic.yaml')]
        )
        #),
        # 4. RCカー足回りドライバ
        #Node(
            #package='rc_car_driver',
            #executable='vehicle_driver',
            #parameters=['config/vehicle_driver.yaml']
        #)
        # ※ autonomous_navigator は run で起動するためここには書かない
    ])
