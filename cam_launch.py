#!/usr/bin/env python3
"""
Standalone Python script to launch all 6 GStreamer camera nodes.
This script uses the ROS 2 launch API but is run directly,
not via `ros2 launch`.
"""

import sys
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

# --- CONFIGURE YOUR CAMERAS HERE ---
#
# List all your cameras. Make sure the 'name' matches the
# keys in your web_server.py config and index.html IDs.
# The 'port' must be the unique UDP port for that camera.
#
camera_configs = [
    {'name': 'front_left',   'port': 6000},
    {'name': 'front_center', 'port': 6001},
    {'name': 'front_right',  'port': 6002},
    {'name': 'rear_left',    'port': 6003},
    {'name': 'rear_center',  'port': 6004},
    {'name': 'rear_right',   'port': 6005},
]
# -----------------------------------


def main():
    # Base GStreamer pipeline string.
    # The '{}' will be replaced with the port number.
    base_pipeline = (
        'udpsrc port={} caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" ! '
        'rtph264depay ! h264parse ! avdec_h264 ! '
        'videoconvert ! video/x-raw,format=BGR'
    )

    # Create a LaunchDescription to hold all our nodes
    ld = LaunchDescription()

    print(f"Preparing to launch {len(camera_configs)} camera nodes...")

    for config in camera_configs:
        cam_name = config['name']
        cam_port = config['port']

        # 1. Format the full pipeline string with the correct port
        pipeline_str = base_pipeline.format(cam_port)

        # 2. Define the *target* relative topic names
        target_image_topic = f"{cam_name}/image_raw"
        target_info_topic = f"{cam_name}/camera_info"
        
        # 3. Create the Node definition
        gscam_node = Node(
            package='gscam',
            executable='gscam_node',
            name=f'{cam_name}_cam_node',
            namespace='cameras',
            output='screen',
            parameters=[
                {'gscam_config': pipeline_str},
                # {'image_topic': topic_name},  # <--- REMOVE THIS LINE
                {'camera_name': cam_name},
                {'camera_info_url': ''}, 
                {'use_gst_timestamps': False},
            ],
            # ADD THIS remappings ARGUMENT
            remappings=[
                ('camera/image_raw', target_image_topic),
                ('camera/camera_info', target_info_topic)
            ]
        )

        # 4. Add the node to our launch description
        ld.add_action(gscam_node)
        # (Optional: Update print statement to be clearer)
        print(f"  - Adding '{cam_name}' (Port: {cam_port}) -> Remapping to: /cameras/{target_image_topic}")

    # Create a LaunchService and run the description
    print("\nStarting launch service... (Press Ctrl+C to stop all nodes)")
    ls = LaunchService()
    ls.include_launch_description(ld)
    
    try:
        # The run() function blocks until shutdown (e.g., Ctrl+C)
        ls.run()
    except KeyboardInterrupt:
        print("Caught Ctrl+C, shutting down nodes...")
    finally:
        # Ensure shutdown is called
        ls.shutdown()


if __name__ == '__main__':
    # Make sure to source your ROS 2 workspace before running this script!
    # e.g., source install/setup.bash
    main()
