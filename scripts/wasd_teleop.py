#!/usr/bin/env python3
"""
ME48B Library Robot - WASD Teleop Controller
Custom keyboard control with WASD keys and adjustable speed

CONTROLS:
  W/S: Forward/Backward
  A/D: Turn Left/Right
  SPACE: Stop
  Q: Increase Speed
  Z: Decrease Speed
  ESC: Quit
"""

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# Control keys
KEYS = {
    'w': (1.0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    'q': (0.0, 0.0),    # Speed Up (handled separately)
    'z': (0.0, 0.0),    # Speed Down (handled separately)
    ' ': (0.0, 0.0),    # Stop
}

# Initial Speed settings (Faster defaults)
LINEAR_SPEED = 0.8      # m/s
ANGULAR_SPEED = 2.5     # rad/s
SPEED_STEP = 0.2

def get_key(timeout=0.1):
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_status():
    print(f"\rSpeed: {LINEAR_SPEED:.1f} m/s | Turn: {ANGULAR_SPEED:.1f} rad/s  ", end='', flush=True)

def main():
    global LINEAR_SPEED, ANGULAR_SPEED
    
    rospy.init_node('wasd_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    print("""
    🎮 ME48B FAST TELEOP 🎮
    -------------------------
    W / S  : Forward / Back
    A / D  : Turn Left / Right
    SPACE  : STOP
    Q / Z  : Speed UP / DOWN
    ESC    : Quit
    """)
    
    print_status()
    twist = Twist()
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key == '\x1b':  # ESC
                break
            
            # Speed Control
            if key.lower() == 'q':
                LINEAR_SPEED = min(LINEAR_SPEED + SPEED_STEP, 2.0)
                ANGULAR_SPEED = min(ANGULAR_SPEED + SPEED_STEP, 4.0)
                print_status()
            elif key.lower() == 'z':
                LINEAR_SPEED = max(LINEAR_SPEED - SPEED_STEP, 0.1)
                ANGULAR_SPEED = max(ANGULAR_SPEED - SPEED_STEP, 0.5)
                print_status()
            
            # Movement Control
            if key.lower() == 'w':
                twist.linear.x = LINEAR_SPEED
                twist.angular.z = 0.0
            elif key.lower() == 's':
                twist.linear.x = -LINEAR_SPEED
                twist.angular.z = 0.0
            elif key.lower() == 'a':
                twist.linear.x = 0.0
                twist.angular.z = ANGULAR_SPEED
            elif key.lower() == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -ANGULAR_SPEED
            else:
                # Stop instantly if key released
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            pub.publish(twist)
            
    except Exception as e:
        print(e)
    finally:
        pub.publish(Twist())
        print("\nStopped.")

if __name__ == '__main__':
    main()
