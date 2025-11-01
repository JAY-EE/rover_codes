#!/usr/bin/env python3
"""
This script launches 6 "fake" GStreamer camera streams.
Each stream is a different test pattern sent to a different
UDP port on localhost.
"""

import subprocess
import time
import sys
import shlex

# This list MUST match your cam_launch.py
camera_configs = [
    {'name': 'front_left',   'port': 6000},
    {'name': 'front_center', 'port': 6001},
    {'name': 'front_right',  'port': 6002},
    {'name': 'rear_left',    'port': 6003},
    {'name': 'rear_center',  'port': 6004},
    {'name': 'rear_right',   'port': 6005},
]

# This pipeline creates a test video, encodes it, and sends it
base_pipeline_template = (
    'gst-launch-1.0 -v videotestsrc pattern={pattern_id} ! '
    'video/x-raw,framerate=20/1 ! videoconvert ! x264enc tune=zerolatency ! '
    'rtph264pay pt=96 ! '
    'udpsink host=127.0.0.1 port={port}'
)

def main():
    processes = []
    print(f"Starting {len(camera_configs)} fake camera senders...")
    
    try:
        for i, config in enumerate(camera_configs):
            port = config['port']
            
            # Use a different test pattern for each camera (0, 1, 2, ...)
            pipeline_str = base_pipeline_template.format(pattern_id=i, port=port)
            
            # shlex.split helps handle quotes safely
            args = shlex.split(pipeline_str)
            
            print(f"  - Launching '{config['name']}' (pattern {i}) to port {port}")
            # Run the command in the background
            proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            processes.append(proc)
            
        print("\nAll fake cameras are running. Press Ctrl+C to stop them.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C, terminating all camera senders...")
        for proc in processes:
            proc.terminate()
        print("Done.")

if __name__ == '__main__':
    main()
