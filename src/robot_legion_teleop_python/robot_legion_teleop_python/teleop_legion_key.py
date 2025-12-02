#!/usr/bin/env python3
"""
teleop_legion_key.py

Educational version of a keyboard teleoperation node for ROS 2.

This ROS2 teleop node:
- Uses custom key bindings (arrow keys, WASD-like layout, etc.).
- Publishes geometry_msgs/Twist to a configurable cmd_vel topic.

Key mapping:

MOVEMENT (Twist linear.x / angular.z)
------------------------------------
Up Arrow      : forward            ( +linear,  0 angular)
Down Arrow    : backward           ( -linear,  0 angular)
Left Arrow    : rotate left        ( 0 linear, +angular)
Right Arrow   : rotate right       ( 0 linear, -angular)

a             : forward + left     ( +linear, +angular)
d             : forward + right    ( +linear, -angular)
<             : backward + right   ( -linear, -angular)
c             : backward + left    ( -linear, +angular)

Space         : stop (all velocities = 0)

SPEED CONTROL (scales, not immediate motion)
--------------------------------------------
w             : increase BOTH linear and angular speed scales
e             : decrease BOTH linear and angular speed scales
q             : increase ONLY linear speed scale
r             : increase ONLY angular speed scale

Notes:
- Speed scales act like global multipliers.
- The node prints the current speed scales in the terminal.
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Helper functions for keyboard handling
# ---------------------------------------------------------------------------

def get_key(settings):
    """
    Read a single key press from stdin in "raw" mode.

    We temporarily put the terminal in raw mode so that:
    - We get key presses immediately (no need to press Enter).
    - Special keys like arrows are delivered as escape sequences.

    This function is modeled on the typical teleop pattern used in ROS tools.
    """
    tty.setraw(sys.stdin.fileno())
    # Use select to implement a small timeout: if no key is pressed,
    # we return an empty string so the main loop can continue.
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    if rlist:
        key = sys.stdin.read(1)
        # Arrow keys come as multi-byte sequences starting with '\x1b'
        if key == '\x1b':
            # Read the next two bytes (this is what arrow keys produce on most terminals)
            key += sys.stdin.read(2)
        return key
    else:
        return ''


def restore_terminal_settings(settings):
    """
    Restore the original terminal settings so that the shell behaves normally
    after the program exits or is interrupted.
    """
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


# ---------------------------------------------------------------------------
# Teleop Node Definition
# ---------------------------------------------------------------------------

class RobotLegionTeleop(Node):
    """
    Node that turns keyboard input into geometry_msgs/Twist messages.

    Main responsibilities:
    - Define which keys correspond to which movement commands.
    - Maintain speed scales for linear and angular velocities.
    - Print helpful instructions and feedback to the user.
    - Publish Twist messages whenever a movement key is pressed.
    """

    def __init__(self):
        super().__init__('robot_legion_teleop_python')

        # Declare a parameter so the cmd_vel topic can be changed at runtime
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        # Publisher for Twist messages
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Initial speed scales (these can be tuned to your liking)
        self.linear_speed = 0.5   # meters per second
        self.angular_speed = 1.0  # radians per second

        # The scale factors that get multiplied/divided when we press speed keys
        self.speed_step = 1.1  # 10% increments

        # Create the movement mapping.
        # Each key maps to a pair: (linear_x_multiplier, angular_z_multiplier)
        #
        # NOTE: For arrow keys we map the escape sequences:
        #  Up Arrow    : '\x1b[A'
        #  Down Arrow  : '\x1b[B'
        #  Right Arrow : '\x1b[C'
        #  Left Arrow  : '\x1b[D'
        #
        # The multipliers are dimensionless; they will be multiplied by
        # self.linear_speed and self.angular_speed.
        self.move_bindings = {
            '\x1b[A': (1, 0),    # Up Arrow      -> forward
            '\x1b[B': (-1, 0),   # Down Arrow    -> backward
            '\x1b[D': (0, 1),    # Left Arrow    -> rotate left
            '\x1b[C': (0, -1),   # Right Arrow   -> rotate right

            'a': (1, 1),         # forward-left
            'd': (1, -1),        # forward-right
            '<': (-1, -1),       # backward-right
            'c': (-1, 1),        # backward-left
        }

        # Speed control bindings.
        # Each entry maps a key to a function that updates linear_speed and/or angular_speed.
        self.speed_bindings = {
            # Increase BOTH linear and angular speeds
            'w': self._increase_both_speeds,

            # Decrease BOTH linear and angular speeds
            'e': self._decrease_both_speeds,

            # Increase ONLY linear speed
            'q': self._increase_linear_speed,

            # Increase ONLY angular speed
            'r': self._increase_angular_speed,
        }

        # Print detailed instructions once at startup
        self._print_instructions(cmd_vel_topic)

    # ----------------------------------------------------------------------
    # Printing / UI helpers
    # ----------------------------------------------------------------------

    def _print_instructions(self, topic_name):
        """
        Print a help banner explaining all keys and the current speed scales.
        """
        print("-----------------------------------------------------------")
        print(" Robot Legion Teleop (Python)")
        print("-----------------------------------------------------------")
        print("Publishing to Twist topic: {}".format(topic_name))
        print("")
        print("MOTION KEYS:")
        print("  Arrow Up        : move forward")
        print("  Arrow Down      : move backward")
        print("  Arrow Left      : rotate left in place")
        print("  Arrow Right     : rotate right in place")
        print("")
        print("  a               : forward + left")
        print("  d               : forward + right")
        print("  <               : backward + right")
        print("  c               : backward + left")
        print("")
        print("  SPACE           : stop (zero linear and angular)")
        print("")
        print("SPEED CONTROL:")
        print("  w               : increase BOTH linear and angular speeds")
        print("  e               : decrease BOTH linear and angular speeds")
        print("  q               : increase ONLY linear speed")
        print("  r               : increase ONLY angular speed")
        print("")
        print("CTRL-C to quit.")
        print("-----------------------------------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        """
        Print the current linear and angular speed scales.
        """
        print("Current speeds -> linear: {:.3f}  angular: {:.3f}".format(
            self.linear_speed, self.angular_speed
        ))

    # ----------------------------------------------------------------------
    # Speed scaling methods (called from speed_bindings)
    # ----------------------------------------------------------------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        print("[w] Increased BOTH speeds by {:.0f}%".format((self.speed_step - 1.0) * 100.0))
        self._print_current_speeds()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        print("[e] Decreased BOTH speeds by {:.0f}%".format((self.speed_step - 1.0) * 100.0))
        self._print_current_speeds()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        print("[q] Increased LINEAR speed by {:.0f}%".format((self.speed_step - 1.0) * 100.0))
        self._print_current_speeds()

    def _increase_angular_speed(self):
        self.angular_speed *= self.speed_step
        print("[r] Increased ANGULAR speed by {:.0f}%".format((self.speed_step - 1.0) * 100.0))
        self._print_current_speeds()

    # ----------------------------------------------------------------------
    # Main loop logic
    # ----------------------------------------------------------------------

    def run(self):
        """
        Main loop: read keys, interpret them, publish Twist messages.

        We don't use rclpy.spin() here because we want tight control over
        the loop and we don't rely on timers or subscriptions.
        """
        # Save the current terminal settings so we can restore them later
        settings = termios.tcgetattr(sys.stdin)

        try:
            while rclpy.ok():
                key = get_key(settings)

                if key == '':
                    # No key pressed during this cycle; just continue.
                    # We do NOT continuously publish the last command here to keep
                    # behavior close to the typical teleop_twist_keyboard model.
                    continue

                # Exit condition: CTRL-C will actually be handled by ROS, but
                # it's useful to also check for '\x03' (Ctrl-C).
                if key == '\x03':
                    break

                # Check if this is a movement key
                if key in self.move_bindings:
                    lin_mult, ang_mult = self.move_bindings[key]

                    twist = Twist()
                    twist.linear.x = self.linear_speed * lin_mult
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0

                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = self.angular_speed * ang_mult

                    self.publisher_.publish(twist)

                # Stop key: space bar
                elif key == ' ':
                    twist = Twist()
                    # All fields default to zero, but we set explicitly for clarity
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0

                    print("[SPACE] Stop command issued.")
                    self.publisher_.publish(twist)

                # Speed adjustment keys
                elif key in self.speed_bindings:
                    self.speed_bindings[key]()

                else:
                    # Unknown key: ignore, but show hint
                    print("Unknown key: {!r}. Press CTRL-C to quit.".format(key))

        except Exception as e:
            # If something unexpected happens, print the exception so it is visible.
            print("Exception in teleop node:", e)

        finally:
            # On exit we send a final zero Twist to ensure the robot is commanded to stop.
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)

            # Restore the original terminal settings so the shell works normally again.
            restore_terminal_settings(settings)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(args=None):
    """
    Standard ROS 2 Python entry point.

    - Initialize the rclpy library.
    - Create the node.
    - Run the teleoperation loop.
    - Clean up once done.
    """
    rclpy.init(args=args)
    node = RobotLegionTeleop()

    try:
        node.run()
    finally:
        # Always destroy the node and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
