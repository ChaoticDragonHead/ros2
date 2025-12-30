#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for multiple robots, with FPV integration:

- Publishes geometry_msgs/Twist to a cmd_vel topic (per-robot).
- Lets you switch robots at runtime with 'm'.
- Publishes the currently controlled robot on /active_robot (std_msgs/String).

Any node can subscribe to /active_robot to "follow" the active robot,
e.g. an FPV camera multiplexer.
"""

import sys
import select
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64 #Imported Float64 to publish float64 values to move joints in the robot
from sensor_msgs.msg import JointState #Joint state publisher


def get_key(settings):
    """
    Read a single key press from stdin in raw mode.
    Arrow keys come in as escape sequences.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # start of escape sequence (arrows, etc.)
            key += sys.stdin.read(2)
        return key
    else:
        return ''


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__('robot_legion_teleop_python')

        # Parameter: default cmd_vel topic
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic')\
            .get_parameter_value().string_value
        self.cmd_vel_topic = cmd_vel_topic

        # Publisher for Twist commands
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)

        #Publisher for arm joint movement
        # Old code for publisher > pusherArmJoint_pub = rospy.Publisher('/potatobot/arm_base_joint_velocity_controller/command', Float64, queue_size=10)
        self.armBaseJointPublisher_ = self.create_publisher(JointState, '/arm_base_joint/cmd_vel', 10)


        self.pusher_velocity = 0.6 #velocity = mps
        self.pusher_limit = 0.1 #limit should be half of arm plus half of pusher length

        #Joint names array
        self.joint_names = ['base_joint','base_right_wheel_joint','base_left_wheel_joint','base_caster_wheel_joint','left_pusher_pusher_arm_joint','right_pusher_pusher_arm_joint','arm_base_joint','camera_joint']

        # Publisher for active robot name
        self.active_robot_pub = self.create_publisher(String, '/active_robot', 10)
        self.current_robot_name: Optional[str] = self._extract_robot_name_from_topic(cmd_vel_topic)
        if self.current_robot_name:
            self._publish_active_robot()

        # Base speed profile (SLOW)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.speed_step = 1.1

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        # Derived profiles
        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)
        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # Last commanded direction
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.is_moving = False

        # Movement bindings: key -> (linear_mult, angular_mult)
        self.move_bindings = {
            '\x1b[A': (1, 0),    # Up arrow
            '\x1b[B': (-1, 0),   # Down arrow
            '\x1b[D': (0, 1),    # Left arrow
            '\x1b[C': (0, -1),   # Right arrow

            '8': (1, 0),   # Numpad up
            '2': (-1, 0),  # Numpad down
            '4': (0, 1),   # Numpad left
            '6': (0, -1),  # Numpad right

            'a': (1, 1),
            'd': (1, -1),
            '<': (-1, -1),
            'c': (1, -1),

            '7': (1, 1),
            '9': (1, -1),
            '1': (-1, 1),
            '3': (-1, -1),
        }

        # Speed and profile bindings: key -> method
        self.speed_bindings = {
            'w': self._increase_both_speeds,
            '+': self._increase_both_speeds,
            'e': self._decrease_both_speeds,
            '-': self._decrease_both_speeds,
            'q': self._increase_linear_speed,
            '/': self._increase_linear_speed,
            'r': self._decrease_linear_speed,
            '*': self._decrease_linear_speed,
            'i': self._set_slow_profile,
            'o': self._set_medium_profile,
            'p': self._set_fast_profile,
        }

        self.joint_keybinds = {
            'j': self._pusher_left,
            'k': self._pusher_center,
            'l': self._pusher_right,
        }

        self._print_instructions(cmd_vel_topic)

    # ------------- Printing & help -------------

    def _print_instructions(self, topic_name):
        print("--------------------------------------------------")
        print(" Robot Legion Teleop (Python) with FPV support")
        print("--------------------------------------------------")
        print(f"Publishing Twist on: {topic_name}")
        if self.current_robot_name:
            print(f"Initial active robot: {self.current_robot_name}")
        print("")
        print("Movement:")
        print("  Arrow keys / numpad 8,2,4,6   : forward/back/turn")
        print("  a,d,<,c and numpad 7,9,1,3    : diagonals")
        print("Stop:")
        print("  [SPACE], 's', or numpad 5     : stop")
        print("")
        print("Speed:")
        print("  w / +   : increase linear+angular")
        print("  e / -   : decrease linear+angular")
        print("  q / /   : increase linear")
        print("  r / *   : decrease linear")
        print("Profiles:")
        print("  i       : SLOW")
        print("  o       : MEDIUM")
        print("  p       : FAST")
        print("")
        print("Robot selection:")
        print("  m       : switch robot (updates /active_robot)")
        print("")
        print("CTRL-C to quit.")
        print("--------------------------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        print("Linear Speed: {:.3f}  Angular Speed: {:.3f}".format(
            self.linear_speed, self.angular_speed))

    # ------------- Active robot helpers -------------

    def _extract_robot_name_from_topic(self, topic: str) -> Optional[str]:
        if not topic:
            return None
        if not topic.startswith('/'):
            topic = '/' + topic
        parts = topic.split('/')
        # ['', 'emiliobot', 'cmd_vel']
        if len(parts) >= 3 and parts[2] == 'cmd_vel' and parts[1]:
            return parts[1]
        if 'cmd_vel' in parts:
            idx = parts.index('cmd_vel')
            if idx > 1 and parts[idx - 1]:
                return parts[idx - 1]
        return None

    def _publish_active_robot(self):
        if not self.current_robot_name:
            print("[ACTIVE ROBOT] No robot name inferred; nothing to publish.")
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        print(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")


    # -----------------------------------------------------------------------
    # Defining joint move functions so we can use keybinds above
    # -----------------------------------------------------------------------
    def _pusher_left(self):
        #Set velocity to -.1 unless joint position is greater than .25
        msg = JointState()
        msg.name = self.joint_names[7]
        msg.velocity = self.pusher_velocity

        print("Publishing to joint " + str(msg.name) + "\n ~~~ \n ~~~ \n Publishing velocity " + str(msg.velocity))
        self.armBaseJointPublisher_.publish(msg)
        print("Joint triggered")

    def _pusher_right(self):
        #Set velocity to -.1 unless joint position is greater than .25
        msg = JointState()
        msg.name = self.joint_names[7]
        msg.velocity = self.pusher_velocity

        print("Publishing to joint " + str(msg.name) + "\n ~~~ \n ~~~ \n Publishing velocity " + str(msg.velocity))
        self.armBaseJointPublisher_.publish(msg)
        print("Joint triggered")
    
    def _pusher_center(self):
        #Set velocity to -.1 unless joint position is greater than .25
        msg = JointState()
        msg.name = self.joint_names[7]
        msg.velocity = self.pusher_velocity

        print("Publishing to joint " + str(msg.name) + "\n ~~~ \n ~~~ \n Publishing velocity " + str(msg.velocity))
        self.armBaseJointPublisher_.publish(msg)
        print("Joint triggered")

    # ------------- Twist republishing -------------

    def _republish_last_twist(self):
        if not self.is_moving:
            return
        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return
        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.angular.z = self.angular_speed * self.last_ang_mult
        self.publisher_.publish(twist)

    # ------------- Speed modifiers -------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        print("[w/+] Increased both speeds.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        print("[e/-] Decreased both speeds.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        print("[q//] Increased linear speed.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        print("[r/*] Decreased linear speed.")
        self._print_current_speeds()
        self._republish_last_twist()

    # ------------- Profiles -------------

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        print("[i] SLOW profile.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        print("[o] MEDIUM profile.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        print("[p] FAST profile.")
        self._print_current_speeds()
        self._republish_last_twist()

    # ------------- Robot switching -------------

    def _switch_robot_prompt(self, settings):
        restore_terminal_settings(settings)
        try:
            print("\n[ROBOT SWITCH] Enter robot name or full cmd_vel topic.")
            print("  Example names: my_robot, emiliobot")
            print("  Example topic: /emiliobot/cmd_vel")
            user_input = input("[ROBOT SWITCH] Target: ").strip()
        except Exception as exc:
            print(f"[ROBOT SWITCH] Input cancelled: {exc}")
            tty.setraw(sys.stdin.fileno())
            return

        tty.setraw(sys.stdin.fileno())

        if not user_input:
            print("[ROBOT SWITCH] No input; keeping:", self.cmd_vel_topic)
            return

        if user_input.startswith('/') or '/' in user_input:
            new_topic = user_input
            new_robot_name = self._extract_robot_name_from_topic(new_topic)
        else:
            new_topic = f"/{user_input}/cmd_vel"
            new_robot_name = user_input

        if not new_topic.startswith('/'):
            new_topic = '/' + new_topic

        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass

        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic
        self.current_robot_name = new_robot_name

        print(f"[ROBOT SWITCH] Now publishing Twist to: {new_topic}")
        self._publish_active_robot()

    # ------------- Main loop -------------

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)
                if key == '':
                    continue

                if key == '\x03':  # Ctrl-C
                    break

                if key in self.move_bindings:
                    lin_mult, ang_mult = self.move_bindings[key]
                    self.last_lin_mult = lin_mult
                    self.last_ang_mult = ang_mult
                    self.is_moving = True

                    twist = Twist()
                    twist.linear.x = self.linear_speed * lin_mult
                    twist.angular.z = self.angular_speed * ang_mult
                    self.publisher_.publish(twist)

                elif key in (' ', '5', 's'):
                    twist = Twist()
                    self.publisher_.publish(twist)
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    print("[STOP]")

                elif key == 'm':
                    self._switch_robot_prompt(settings)

                elif key in self.speed_bindings:
                    self.speed_bindings[key]()

                elif key in ('j', 'k', 'l'):
                    print("pusherarm triggered")
                    #publishing message to joint
                    #self.armBaseJointPublisher_.publish(velocity=1) < does not work "Exception in teleop: Publisher.publish() got an unexpected keyword argument 'velocity'"
                    #need to add function here

                else:
                    print(f"Unknown key: {repr(key)} (CTRL-C to quit).")

        except Exception as e:
            print("Exception in teleop:", e)
        finally:
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)
            restore_terminal_settings(settings)


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
