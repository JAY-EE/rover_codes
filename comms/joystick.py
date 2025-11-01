
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import sys


# Initialize Pygame and ROS 2
pygame.init()
pygame.joystick.init()
rclpy.init(args=None)
node = rclpy.create_node('Client_Server')

# Global variables 

joysticks = []

kfront = 100
kleftright = 175
shoulder = 0
elbow = 0
base = 0
linear = 0 
angular = 0
msg = 0


msg = {
    "linear":0,
    "angular":0,
    "shoulder":0,
    "elbow":0,
    "base":0
}


publisher_msg = node.create_publisher(String, '/arm_rover', 10)

def setup_joystick():
    global joystick

    joystick_count = pygame.joystick.get_count()
    print(f"Joystick count: {joystick_count}")

    if joystick_count == 1:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick detected: {joystick.get_name()}")
    else:
        print("No Joysticks found.")
        pygame.quit()
        sys.exit()


def process_events():

    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            joystick = pygame.joystick.Joystick(event.device_index)
            joystick.init()
            print(f"Joystick {joystick.get_name()} added.")
            setup_joystick()


def update_joystick():  # set the joystick buttons ..
    
    global kfront, kleftright, msg, shoulder, elbow, base
    global linear, angular, ac1_fwd, ac1_rev, ac2_fwd, ac2_rev, base_speed


    ac1_fwd = joystick.get_button(6)
    ac1_rev = joystick.get_button(7)
    ac2_fwd = joystick.get_button(8)
    ac2_rev = joystick.get_button(9)
    base = joystick.get_axis(2)
       


    frontback = joystick.get_axis(1)
    rightleft = joystick.get_axis(0)

    if joystick.get_button(1) == 1:    # increase the pwm of rover 
        kfront += 2 if kfront<254 else 0
        kleftright += 1 if kfront<254 else 0

    if joystick.get_button(3) == 1 and kfront > 1:
        kfront -= 2
        kleftright -= 1 if kleftright > 1 else 0

    if abs(frontback) > 0.2:
        linear = float(-frontback * kfront)
    else:
        linear = 0.0

    if abs(rightleft) > 0.2:
        angular = float(-rightleft * kleftright)
    else:
        angular = 0.0


def handle_input():

    global ac1_fwd, ac1_rev, ac2_rev, ac2_fwd, shoulder, elbow, base
  
    if ac1_fwd > 0:
        shoulder = 1
    elif ac1_rev == 1:
        shoulder = -1
    elif ac2_fwd > 0:
        elbow = 1
    elif ac2_rev == 1:
        elbow = -1
    else:
        shoulder =0 
        elbow = 0

    if (base  > 0.5):
        base = 1
    elif (base < -0.5):
        base = -1
    else:
        base = 0


def publish_data():
    global msg, publisher_msg

    msg = {
    "linear":linear,
    "angular":angular,
    "shoulder":shoulder,
    "elbow":elbow,
    "base":base
    }

    msg_json = json.dumps(msg)
    pub_msg  = String()
    pub_msg.data = msg_json
    publisher_msg.publish(pub_msg)
   
    print(msg)

def main_loop():

    while rclpy.ok():
        pygame.event.pump()

        update_joystick()

        handle_input()

        publish_data()

        process_events()

        rclpy.spin_once(node, timeout_sec= 0.01)

        # Exit condition
        if joysticks and joysticks[0].get_button(10):
            print("Tata Bye Bye ... See You Soon Again !!!")
            break

    # Shutdown once loop exits
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


def main():
    global joystick_arm, joystick_rover
    setup_joystick()
    main_loop()

    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()

