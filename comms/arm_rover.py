import serial
import serial.tools.list_ports
import json
import time
import asyncio
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ===========================
# Global Variables
# ===========================
linear = 0
angular = 0
shoulder = 0
elbow = 0
base = 0
serial_connection = None


# ===========================
# ROS Callback
# ===========================
def callback(message):  # Read values from /cmd_vel
    global linear, angular, shoulder, elbow, base
    
    msg = json.loads(message.data)


    linear= msg.get('linear',0)
    angular = msg.get('angular',0)
    shoulder = msg.get('shoulder',0)
    elbow = msg.get('elbow',0)
    base = msg.get('base',0)




# ===========================
# Serial Port Scanner
# ===========================
def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            try:
                ser = serial.Serial(port.device, 115200, timeout=None)
                time.sleep(2)
                ser.write(json.dumps({"command": "identify"}).encode() + b'\n')
                try:
                    response = ser.readline().decode().strip()
                    data = json.loads(response)
                    if data.get("device_type") == "arm_rover":
                        print(f"Connected to {port.device}")
                        return ser
                except (UnicodeDecodeError, json.JSONDecodeError):
                    pass
                ser.close()
            except serial.SerialException:
                pass
    return None


# ===========================
# Main Async Loop
# ===========================
async def main_async():
    global serial_connection

    rclpy.init()
    node = Node('arm_rover')
    node.create_subscription(String, '/cmd_vel', callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if serial_connection is None:
                print("Searching for the rover ..")
                serial_connection = find_serial_port()
                if serial_connection:
                    print("Rover connected!")
                else:
                    print("Rover not found. Retrying ...")
                    await asyncio.sleep(2)
                    continue

            try:
                outgoing_data = {
                    "linear": float(linear),
                    "angular": float(angular),
                    "shoulder": float(shoulder),
                    "elbow": float(elbow),
                    "base ": float(base),
                }
                # print(outgoing_data)
                serial_connection.write((json.dumps(outgoing_data) + '\n').encode())

                # Receive data from rover
                if serial_connection.in_waiting > 0:
                    line = serial_connection.readline().decode(errors='ignore').strip()
                    if line:
                        try:
                            incoming_data = json.loads(line)
                            a = incoming_data.get('linear', 0)
                            b = incoming_data.get('angular', 0)
                            c = incoming_data.get('shoulder', 0)
                            d = incoming_data.get('elbow', 0)
                            e = incoming_data.get('base', 0)
                            print(f"Linear: {a}, Angular: {b}, Shoulder: {c}, Elbow: {d}, Base: {e}")
                        except json.JSONDecodeError as e:
                            print(f"JSON Error: {e}")
                            print(f"Raw data: {line}")

            except serial.SerialException:
                print("Serial connection error. Reconnecting...")
                if serial_connection:
                    serial_connection.close()
                serial_connection = None

            await asyncio.sleep(0.01)

    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == '__main__':
    main()
