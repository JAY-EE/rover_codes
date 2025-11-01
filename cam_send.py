#!/usr/bin/env python3
"""
Dynamic GStreamer SENDER Script.

This script runs on the robot (with the cameras). It requires NO ROS.
It monitors for /dev/video* devices using pyudev.
When a camera is connected, it finds an available camera slot and
launches a GStreamer pipeline to ENCODE and STREAM the video
to the receiver's IP address.
"""

import sys
import subprocess
import time
import atexit
import pyudev
from pyudev import Context, Monitor, MonitorObserver

# --- CONFIGURE YOUR STREAMING HERE ---

# Set this to the IP address of your Ground Station / Receiver PC
RECEIVER_IP = "localhost" 

# Set this to your desired bitrate in Kbps (kilobits-per-second).
# 1000 = 1 Mbps (good quality)
# 500 = 0.5 Mbps (ok quality, for limited bandwidth)
# 250 = 0.25 Mbps (low quality, for very limited bandwidth)
VIDEO_BITRATE = 500

# This list defines all available "slots" for cameras.
# The 'port' MUST match the ports on your receiver.
CAMERA_CONFIGS = [
    {'name': 'front_left',   'port': 6000},
    {'name': 'front_center', 'port': 6001},
    {'name': 'front_right',  'port': 6002},
    {'name': 'rear_left',    'port': 6003},
    {'name': 'rear_center',  'port': 6004},
    {'name': 'rear_right',   'port': 6005},
]
# -----------------------------------

# --- GStreamer Pipeline Definition ---
# {device}, {port}, {host}, and {bitrate} will be replaced.
BASE_GST_SENDER_CMD = [
    'gst-launch-1.0',
    'v4l2src', 'device={device}', 'do-timestamp=true',
    '!', 'video/x-raw,format=YUY2,width=640,height=480,framerate=30/1',
    '!', 'videoconvert',
    '!', 'x264enc',
    'tune=zerolatency',       # Critical for low latency
    'speed-preset=fast',      # 'fast' is a good CPU/compression trade-off.
    'bitrate={bitrate}',      # The most important setting for bandwidth!
    'key-int-max=60',         # Increases compression (sends keyframes less often)
    'bframes=0',              # B-frames add latency
    'ref=3',                  # Improves compression
    'sliced-threads=true',
    'byte-stream=true',
    '!', 'rtph264pay', 'pt=96', 'mtu=1200', 'config-interval=1',
    '!', 'udpsink',
    'host={host}', 'port={port}',
    'sync=false', 'async=false'
]
# -----------------------------------

# Global dictionary to hold the state of our camera slots
camera_slots = {}


def initialize_slots():
    """Populates the global camera_slots dict from the config."""
    global camera_slots
    for config in CAMERA_CONFIGS:
        name = config['name']
        camera_slots[name] = {
            'name': name,
            'port': config['port'],
            'status': 'empty',      # 'empty' or 'occupied'
            'device_node': None,    # e.g., /dev/video0
            'gst_process': None,    # Popen handle for gst-launch
        }
    print(f"Initialized {len(camera_slots)} camera sender slots.")


def device_event(device, action=None):
    """Callback function for pyudev device events."""
    event_action = action or device.action
    if not event_action:
        return

    # Check if it's a video capture device
    capabilities = device.properties.get("ID_V4L_CAPABILITIES", "")
    if "capture" not in capabilities:
        return

    print(f"\n--- Event: {event_action.upper()} on device {device.device_node} ---")

    if event_action == "add":
        handle_device_add(device)
    elif event_action == "remove":
        handle_device_remove(device)


def handle_device_add(device):
    """Handles a "add" event for a video device."""
    global camera_slots
    
    time.sleep(2) 

    # 1. Find the first available 'empty' slot
    target_slot = None
    for slot in camera_slots.values():
        if slot['status'] == 'empty':
            target_slot = slot
            break

    if not target_slot:
        print(f"Error: All {len(camera_slots)} camera slots are full. "
              f"Cannot assign {device.device_node}.")
        return

    port = target_slot['port']
    name = target_slot['name']
    device_node = device.device_node

    print(f"Assigning {device_node} to slot '{name}' (Port: {port})")

    try:
        # 2. Start the GStreamer SENDER pipeline
        print(f"  > Starting GStreamer sender for {device_node}...")
        print(f"  > Streaming to {RECEIVER_IP}:{port} at {VIDEO_BITRATE} Kbps")
        
        gst_cmd = [arg.format(device=device_node, 
                             port=port,
                             host=RECEIVER_IP,
                             bitrate=VIDEO_BITRATE) 
                   for arg in BASE_GST_SENDER_CMD]
        
        # Use stdout=subprocess.DEVNULL to hide GStreamer's console spam
        gst_proc = subprocess.Popen(gst_cmd, 
                                    stdout=subprocess.DEVNULL, 
                                    stderr=subprocess.PIPE)

        # 3. Update the slot's state
        target_slot.update({
            'status': 'occupied',
            'device_node': device_node,
            'gst_process': gst_proc,
        })
        print(f"Successfully launched sender for slot '{name}'.")

    except Exception as e:
        print(f"Error launching GStreamer sender for slot '{name}': {e}")
        if 'gst_proc' in locals() and gst_proc.poll() is None:
            gst_proc.terminate()
        target_slot['status'] = 'empty' # Reset slot


def handle_device_remove(device):
    """Handles a "remove" event for a video device."""
    global camera_slots

    # 1. Find the slot associated with this device node
    target_slot = None
    for slot in camera_slots.values():
        if slot['device_node'] == device.device_node:
            target_slot = slot
            break

    if not target_slot:
        print(f"Warning: Removed device {device.device_node} was not tracked. Ignoring.")
        return

    print(f"Removing camera {device.device_node} from slot '{target_slot['name']}'...")

    try:
        # 2. Terminate the GStreamer SENDER
        if target_slot['gst_process']:
            print(f"  > Terminating GStreamer sender (PID: {target_slot['gst_process'].pid})...")
            target_slot['gst_process'].terminate()
            target_slot['gst_process'].wait(timeout=2)
            
    except subprocess.TimeoutExpired:
        print("  > Warning: Process did not terminate gracefully. Killing.")
        if target_slot['gst_process'] and target_slot['gst_process'].poll() is None:
            target_slot['gst_process'].kill()
    except Exception as e:
        print(f"Error during process termination: {e}")
    
    finally:
        # 3. Reset the slot to 'empty'
        print(f"Slot '{target_slot['name']}' is now empty.")
        target_slot.update({
            'status': 'empty',
            'device_node': None,
            'gst_process': None,
        })


def shutdown_all_processes():
    """Cleans up all running subprocesses on script exit."""
    print("\nShutting down all active sender processes...")
    global camera_slots
    for slot in camera_slots.values():
        if slot['status'] == 'occupied':
            print(f"Cleaning up slot '{slot['name']}'...")
            if slot['gst_process'] and slot['gst_process'].poll() is None:
                slot['gst_process'].terminate()
                slot['gst_process'].wait()
    print("Shutdown complete.")


def main():
    initialize_slots()
    atexit.register(shutdown_all_processes)

    context = Context()
    monitor = Monitor.from_netlink(context)
    monitor.filter_by(subsystem='video4linux')
    
    observer = MonitorObserver(monitor, callback=device_event)
    observer.start()
    
    print("\n--- Initial Device Scan ---")
    try:
        for device in context.list_devices(subsystem='video4linux'):
            device_event(device, action="add")
    except Exception as e:
        print(f"Error during initial scan: {e}")
        
    print("----------------------------")
    print(f"\nMonitoring for camera hot-plug events...")
    print(f"Streaming to {RECEIVER_IP} at {VIDEO_BITRATE} Kbps.")
    print("(Press Ctrl+C to exit)")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C. Initiating shutdown...")
        sys.exit(0)


if __name__ == '__main__':
    main()