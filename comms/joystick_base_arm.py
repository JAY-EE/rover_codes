# CLIENT (RUNS ON BASE)

""" 
Setup the communication from Joystick

# Publish :

1) Arm josytick values
   Arm ---> arm_client

2) Rover jostick commands
   Rover ---> rover_client

"""

""""
Things to remember :
i) After connecting the joysticks check that arm joystick goes for arm and rover for rover.
ii) If commands get inversed just change the order in which josyticks were connected ..
    First put rover then arm or vice versa.

iii) If still doesnt work .. BE PATIENT .. (try this at last)
     Go to line 109 and change the joystick index i.e.
     In if else .. flip the 0 to 1 and 1 to 0 

     treat it as the final option !!!!!  

"""


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
frontback = 0
rightleft = 0
msg_rover = 0
msg_arm = 0


msg_science = {
    "pump1_pwm":0,
    "pump2_pwm":0,
    "step_fwd":0,
    "step_rev":0
}


msg_arm = {
    "y":0,
    "command":0,
    "position":0,
    "pitch":0,
    "yaw":0,
    "gripper":0,
    "base":0
}


y = 0
x = 0
position = 0
command = 'c'
pitch = 0
yaw = 0
gripper = 0  
base = 0
base_speed = 0

ac1_fwd = 0
ac1_rev = 0
ac2_fwd = 0
ac2_rev = 0

pump1_pwm = 0
pump2_pwm = 0
stepper_fwd = 0
stepper_rev = 0

step_fwd_counter = 0
step_rev_counter = 0
 
dpad = 0
up_down = 0
roll = 0

grip_open = 0
grip_close = 0

joystick_arm = None
joystick_rover = None

# Initialize publishers
publisher_rover = node.create_publisher(Twist, '/rover_client', 10)
publisher_arm = node.create_publisher(String, '/arm_client', 10)
publisher_science = node.create_publisher(String,'/science_client',10)


def setup_joystick():
    global joystick_arm, joystick_rover

    joystick_count = pygame.joystick.get_count()
    print(f"Joystick count: {joystick_count}")

    if joystick_count == 1:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick detected: {joystick.get_name()}")

        if joystick.get_name() == "Sony Interactive Entertainment Wireless Controller" or "Wireless Control":
            joystick_rover = joystick   #None
            joystick_arm = None         #joystick
        elif joystick.get_name() == "Logitech Extreme 3D pro":
            joystick_arm = joystick
            joystick_rover = None
        else:
            print("Unexpected joystick name !!")
            sys.exit()

    elif joystick_count > 1:
        joysticks = [pygame.joystick.Joystick(i) for i in range(2)]
        joysticks[0].init()
        joysticks[1].init()

        print(f"Joystick 1 detected: {joysticks[0].get_name()}")
        print(f"Joystick 2 detected: {joysticks[1].get_name()}")

        if joysticks[0].get_name() == "Sony Interactive Entertainment Wireless Controller" or "Wireless Controller":
            joystick_rover = joysticks[0]
            joystick_arm = joysticks[1]
        else:
            joystick_arm = joysticks[0]
            joystick_rover = joysticks[1]

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
    
    global kfront, kleftright, msg_rover, x , ac1_fwd, ac1_rev, ac2_fwd, ac2_rev, stepper_fwd, stepper_rev
    global up_down, roll, grip_close, grip_open, base, base_speed, pump1_pwm , pump2_pwm

    msg_rover = Twist()

    if joystick_arm:
        x = joystick_arm.get_axis(1)
        ac1_fwd = joystick_arm.get_button(4)
        ac1_rev = joystick_arm.get_button(2)
        ac2_fwd = joystick_arm.get_button(5)
        ac2_rev = joystick_arm.get_button(3)
        dpad = joystick_arm.get_hat(0)
        dpad_y, dpad_x = dpad
        up_down = dpad_y
        roll = dpad_x
        grip_open = joystick_arm.get_button(0)
        grip_close = joystick_arm.get_button(1)
        base = joystick_arm.get_axis(2)
        base_speed = -(joystick_arm.get_axis(3))
        base_speed = (base_speed + 1)*50
       

    if joystick_rover:
        frontback = joystick_rover.get_axis(1)
        rightleft = joystick_rover.get_axis(0)
        pump1_pwm = joystick_rover.get_button(2)
        pump2_pwm = joystick_rover.get_button(0)
        stepper_fwd = joystick_rover.get_button(1)
        stepper_rev = joystick_rover.get_button(3)

        if joystick_rover.get_button(4) == 1:    # increase the pwm of rover 
            kfront += 2 if kfront<254 else 0
            kleftright += 1 if kfront<254 else 0

        if joystick_rover.get_button(5) == 1 and kfront > 1:
            kfront -= 2
            kleftright -= 1 if kleftright > 1 else 0

        if abs(frontback) > 0.2:
            msg_rover.linear.x = float(-frontback * kfront)
        else:
            msg_rover.linear.x = 0.0

        if abs(rightleft) > 0.2:
            msg_rover.angular.z = float(-rightleft * kleftright)
        else:
            msg_rover.angular.z = 0.0

        rover_msg = {
            "linear": msg_rover.linear.x,
            "angular": msg_rover.angular.z
        }

        print(rover_msg)


def handle_input():
    global y, pitch, yaw, gripper, base, command, pump1_pwm, pump2_pwm, stepper_rev, stepper_fwd, msg_science
    global step_fwd_counter, step_rev_counter
  
    if ac1_fwd > 0:
        y = 1
    elif ac1_rev == 1:
        y = 2
    elif ac2_fwd > 0:
        y = 3
    elif ac2_rev == 1:
        y = 4
    else:
        y = 0

    if up_down == -1:
        pitch = 1
    elif up_down == (1):
        pitch = -1
    else:
        pitch = 0

    if roll == -1:
        yaw = 1
    elif roll == (1):
        yaw = -1
    else:
        yaw = 0

    if grip_open == 1:
        gripper = 1
    elif grip_close == 1:
        gripper = -1
    else:
        gripper = 0

    if base > 0.5:
        base = 1*base_speed
    elif base < -0.5:
        base = -1*base_speed
    else:
        base = 0

    if x > 0.5:
        command = 's'
    elif x < -0.5:
        command = 'w'
    else:
        command = 'c'


    if pump1_pwm == 1:
        msg_science["pump1_pwm"] = 1
    else:
        msg_science["pump1_pwm"] = 0
    if pump2_pwm == 1:
        msg_science["pump2_pwm"] = 1
    else:
        msg_science["pump2_pwm"] = 0
    if stepper_fwd == 1:
        msg_science["step_fwd"] = 1
    else:
        msg_science["step_fwd"] = 0
    if stepper_rev == 1:
        msg_science["step_rev"] = 1
    else:
        msg_science["step_rev"] = 0



def publish_data():
    global msg_rover, msg_arm, msg_science

    publisher_rover.publish(msg_rover)

    science_json = json.dumps(msg_science)
    science_pub_msg  = String()
    science_pub_msg.data = science_json
    publisher_science.publish(science_pub_msg)

    msg_arm = {
        "y":int(y),
        "command":ord(command),
        "position":position,
        "pitch":int(pitch),
        "yaw":int(yaw),
        "gripper":int(gripper),
        "base":int(base)
    }

    arm_json = json.dumps(msg_arm)
    arm_pub_msg  = String()
    arm_pub_msg.data = arm_json
    publisher_arm.publish(arm_pub_msg)
   
    print(msg_science)


    if joystick_arm:
        print(msg_arm)
        

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

