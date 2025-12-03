#!/usr/bin/env python3
"""
teleop_legion_key.py

This ROS2 teleop node:
- Uses custom key bindings (arrow keys, WASD-like layout, numpad, etc.).
- Publishes geometry_msgs/Twist to a configurable cmd_vel topic.

Key mapping (high level):

MOVEMENT (Twist linear.x / angular.z)
------------------------------------
Up Arrow / Numpad 8 : forward            ( +linear,  0 angular)
Down Arrow / Numpad 2 : backward         ( -linear,  0 angular)
Left Arrow / Numpad 4 : rotate left      ( 0 linear, +angular)
Right Arrow / Numpad 6 : rotate right    ( 0 linear, -angular)

a / Numpad 7         : forward + left    ( +linear, +angular)
d / Numpad 9         : forward + right   ( +linear, -angular)
< / Numpad 3         : backward + right  ( -linear, -angular)
c / Numpad 1         : backward + left   ( -linear, +angular)

Space / Numpad 5 / s : stop (all velocities = 0)

SPEED CONTROL (scales, not immediate motion by themselves)
----------------------------------------------------------
w / Numpad +         : increase BOTH linear and angular speed 
e / Numpad -         : decrease BOTH linear and angular speed 
q / '/'              : increase ONLY linear speed scale
r / '*'              : decrease ONLY linear speed scale

MOTION PROFILES (predefined speed presets)
------------------------------------------
i : SLOW profile
    - The baseline speed the node starts with.
o : MEDIUM profile
    - Equivalent to pressing '+' 10 times from the slow profile.
p : FAST profile
    - Equivalent to pressing '+' 10 times and '/' 5 times from the slow profile.

Notes:
- Speed scales act like global multipliers.
- The node prints the current speed scales in the terminal.
- Speed and profile changes take effect immediately while moving.
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
    - Provide speed scaling and motion profiles (slow/medium/fast).
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

        # ------------------------------------------------------------------
        # Base speed scales (these define your SLOW profile)
        # ------------------------------------------------------------------
        self.linear_speed = 0.5   # meters per second (slow profile baseline)
        self.angular_speed = 1.0  # radians per second (slow profile baseline)

        # The scale factor used when pressing w/e/q/r, +, -, /, *
        # Each press multiplies/divides speed by this factor.
        self.speed_step = 1.1  # 10% increments

        # Store the "slow profile" baseline so that profiles can be recomputed
        # even if you later change these values.
        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        # ------------------------------------------------------------------
        # Motion profiles
        #
        # These are defined in terms of the slow profile and speed_step so
        # you can easily adjust how "far apart" they are:
        #
        # - MEDIUM profile: slow * (speed_step ** 10)
        # - FAST profile:
        #       linear  = slow_linear  * (speed_step ** (10 + 5))  # 10 both + 5 linear-only
        #       angular = slow_angular * (speed_step ** 10)
        #
        # If you want different spacing, simply change the exponents below.
        # ------------------------------------------------------------------
        self.medium_linear = self.base_slow_linear * (self.speed_step ** 10)
        self.medium_angular = self.base_slow_angular * (self.speed_step ** 10)

        self.fast_linear = self.base_slow_linear * (self.speed_step ** 15)
        self.fast_angular = self.base_slow_angular * (self.speed_step ** 10)

        # Remember the last commanded direction (multipliers) so that
        # speed changes and profile changes can take effect immediately while moving.
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.is_moving = False

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
            # Arrow keys
            '\x1b[A': (1, 0),    # Up Arrow      -> forward
            '\x1b[B': (-1, 0),   # Down Arrow    -> backward
            '\x1b[D': (0, 1),    # Left Arrow    -> rotate left
            '\x1b[C': (0, -1),   # Right Arrow   -> rotate right

            # Numpad cardinal directions (NumLock ON)
            '8': (1, 0),         # Numpad 8 -> forward
            '2': (-1, 0),        # Numpad 2 -> backward
            '4': (0, 1),         # Numpad 4 -> rotate left
            '6': (0, -1),        # Numpad 6 -> rotate right

            # Diagonals: letters and numpad
            'a': (1, 1),         # forward-left
            'd': (1, -1),        # forward-right
            '<': (-1, -1),       # backward-right
            'c': (-1, 1),        # backward-left

            '7': (1, 1),         # Numpad 7 -> forward-left
            '9': (1, -1),        # Numpad 9 -> forward-right
            '1': (-1, 1),        # Numpad 1 -> backward-left
            '3': (-1, -1),       # Numpad 3 -> backward-right
        }

        # Speed control bindings.
        # Each entry maps a key to a function that updates linear_speed and/or angular_speed.
        self.speed_bindings = {
            # Increase BOTH linear and angular speeds
            'w': self._increase_both_speeds,
            '+': self._increase_both_speeds,   # Numpad +

            # Decrease BOTH linear and angular speeds
            'e': self._decrease_both_speeds,
            '-': self._decrease_both_speeds,   # Numpad -

            # Increase ONLY linear speed
            'q': self._increase_linear_speed,
            '/': self._increase_linear_speed,

            # Decrease ONLY linear speed
            'r': self._decrease_linear_speed,
            '*': self._decrease_linear_speed,

            # Motion profiles: slow / medium / fast
            'i': self._set_slow_profile,
            'o': self._set_medium_profile,
            'p': self._set_fast_profile,
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
        print("--------------------------------------------------")
        print(" Robot Legion Teleop (Python)")
        print("--------------------------------------------------")
        print("Publishing to Twist topic: {}".format(topic_name))
        print("")
        print("MOTION KEYS:")
        print("  [UP] / Numpad 8    : move forward")
        print("  [DOWN] / Numpad 2  : move backward")
        print("  [LEFT] / Numpad 4  : rotate left in place")
        print("  [RIGHT] / Numpad 6 : rotate right in place")
        print("")
        print("  a / Numpad 7       : forward + left")
        print("  d / Numpad 9       : forward + right")
        print("  < / Numpad 3       : backward + right")
        print("  c / Numpad 1       : backward + left")
        print("")
        print("  [SPACE] / Numpad 5 / s : stop (zero linear and angular)")
        print("")
        print("SPEED CONTROL:")
        print("  w / Numpad +       : increase linear + angular speed")
        print("  e / Numpad -       : decrease linear + angular speed")
        print("  q / '/'            : increase linear speed")
        print("  r / '*'            : decrease linear speed")
        print("")
        print("MOTION PROFILES (predefined speed presets):")
        print("  i : SLOW    (baseline profile at startup)")
        print("  o : MEDIUM  (slow * speed_step^10)")
        print("  p : FAST    (slow * speed_step^(10+5) for linear, speed_step^10 for angular)")
        print("")
        print("CTRL-C to quit.")
        print("--------------------------------------------------")
        self._print_current_speeds()

    def _print_current_speeds(self):
        """
        Print the current linear and angular speed scales.
        """
        print("Linear Speed: {:.3f}  Angular Speed: {:.3f}".format(
            self.linear_speed, self.angular_speed
        ))

    def _republish_last_twist(self):
        """
        Re-publish the last commanded direction using the *current* speed scales.

        This is used so that speed changes (w/e/q/r, numpad +/- etc.)
        and profile changes (i/o/p) take effect immediately while the
        robot is already moving.
        """
        if not self.is_moving:
            return

        # If both multipliers are zero, there is nothing meaningful to send.
        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return

        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_speed * self.last_ang_mult

        self.publisher_.publish(twist)

    # ----------------------------------------------------------------------
    # Speed scaling methods (called from speed_bindings)
    # ----------------------------------------------------------------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        print("[w / +] Increased BOTH speeds by {:.0f}%".format(
            (self.speed_step - 1.0) * 100.0
        ))
        self._print_current_speeds()
        self._republish_last_twist()

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        print("[e / -] Decreased BOTH speeds by {:.0f}%".format(
            (self.speed_step - 1.0) * 100.0
        ))
        self._print_current_speeds()
        self._republish_last_twist()

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        print("[q / '/'] Increased LINEAR speed by {:.0f}%".format(
            (self.speed_step - 1.0) * 100.0
        ))
        self._print_current_speeds()
        self._republish_last_twist()

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        print("[r / '*'] Decreased LINEAR speed by {:.0f}%".format(
            (self.speed_step - 1.0) * 100.0
        ))
        self._print_current_speeds()
        self._republish_last_twist()

    # ----------------------------------------------------------------------
    # Motion profile methods (slow / medium / fast)
    # ----------------------------------------------------------------------

    def _set_slow_profile(self):
        """
        Set speeds to the SLOW profile.

        By default, this is exactly the baseline speed at startup:
        linear_speed  = base_slow_linear
        angular_speed = base_slow_angular

        To adjust the slow profile, simply change base_slow_linear and/or
        base_slow_angular in __init__.
        """
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        print("[i] Switched to SLOW profile.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_medium_profile(self):
        """
        Set speeds to the MEDIUM profile.

        Defined as what you'd get by pressing '+' 10 times starting
        from the slow profile:

            linear_speed  = base_slow_linear  * (speed_step ** 10)
            angular_speed = base_slow_angular * (speed_step ** 10)

        To adjust "how far" medium is from slow, change the exponent
        used to compute medium_linear / medium_angular in __init__.
        """
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        print("[o] Switched to MEDIUM profile.")
        self._print_current_speeds()
        self._republish_last_twist()

    def _set_fast_profile(self):
        """
        Set speeds to the FAST profile.

        Defined as what you'd get by pressing '+' 10 times and then
        '/' 5 times starting from the slow profile:

            After '+' x10:
                linear_speed  = base_slow_linear  * (speed_step ** 10)
                angular_speed = base_slow_angular * (speed_step ** 10)

            After '/' x5 (linear-only increase):
                linear_speed  = base_slow_linear  * (speed_step ** (10 + 5))
                angular_speed = base_slow_angular * (speed_step ** 10)

        To adjust this profile, change the exponents used to compute
        fast_linear and fast_angular in __init__.
        """
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        print("[p] Switched to FAST profile.")
        self._print_current_speeds()
        self._republish_last_twist()

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
                    continue

                # Exit condition: CTRL-C will actually be handled by ROS, but
                # it's useful to also check for '\x03' (Ctrl-C).
                if key == '\x03':
                    break

                # Check if this is a movement key
                if key in self.move_bindings:
                    lin_mult, ang_mult = self.move_bindings[key]

                    # Remember the last direction so speed changes can reuse it
                    self.last_lin_mult = lin_mult
                    self.last_ang_mult = ang_mult
                    self.is_moving = True

                    twist = Twist()
                    twist.linear.x = self.linear_speed * lin_mult
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0

                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = self.angular_speed * ang_mult

                    self.publisher_.publish(twist)

                # Stop keys: space bar, numpad 5, or 's'
                elif key in (' ', '5', 's'):
                    twist = Twist()
                    # All fields default to zero, but we set explicitly for clarity
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0

                    print("[STOP] Stop command issued (key: {!r}).".format(key))
                    self.publisher_.publish(twist)

                    # Mark that we are no longer moving
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0

                # Speed adjustment keys (including profiles)
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
