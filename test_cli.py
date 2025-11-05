#!/usr/bin/env python3
"""
Interactive CLI for testing Jetson control node.

Usage:
    python test_cli.py [--port /dev/ttyUSB0] [--baudrate 921600]

Controls:
    W/S: Increase/Decrease speed
    A/D: Increase/Decrease steering angle
    M: Toggle AUTO/MANUAL mode
    R: Arm system
    U: Disarm system
    L: Toggle lights
    E: Emergency brake
    Q: Quit
"""

import argparse
import sys
import select
import termios
import tty
import time
import threading
from jetson_control import JetsonControlNode, SystemMode


class KeyboardMonitor:
    """Monitor keyboard input in non-blocking mode."""
    
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
    
    def __enter__(self):
        return self
    
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self, timeout=0.1):
        """Get a key press, return None if timeout."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None


def print_help():
    """Print control help."""
    print("\n" + "="*60)
    print("Jetson Control Node - Interactive CLI")
    print("="*60)
    print("Controls:")
    print("  W/S        : Increase/Decrease speed")
    print("  A/D        : Increase/Decrease steering angle")
    print("  M          : Toggle AUTO/MANUAL mode")
    print("  R          : Arm system")
    print("  U          : Disarm system")
    print("  L          : Toggle lights")
    print("  E          : Emergency brake")
    print("  Q          : Quit")
    print("  H/?        : Show this help")
    print("="*60 + "\n")


def print_status(node: JetsonControlNode, speed: float, angle: float, mode: SystemMode, armed: bool, lights: bool):
    """Print current status."""
    stats = node.get_stats()
    print(f"\rSpeed: {speed:6.2f} | Angle: {angle:6.2f} | Mode: {mode.name:6s} | "
          f"Armed: {'YES' if armed else 'NO':3s} | Lights: {'ON' if lights else 'OFF':3s} | "
          f"Sent: {stats['sent_count']:5d} | Rcvd: {stats['received_count']:5d} | "
          f"CRC Errors: {stats['crc_errors']:3d}", end='', flush=True)


def interactive_control(node: JetsonControlNode):
    """Run interactive control loop."""
    speed = 0.0
    angle = 0.0
    speed_step = 0.1
    angle_step = 5.0
    lights_on = False
    
    print_help()
    
    with KeyboardMonitor() as kb:
        print_status(node, speed, angle, node.system_mode, node.armed, lights_on)
        
        while True:
            key = kb.get_key(timeout=0.1)
            
            if key is None:
                # Update control even if no key pressed
                node.set_control(speed, angle)
                print_status(node, speed, angle, node.system_mode, node.armed, lights_on)
                continue
            
            key = key.lower()
            
            if key == 'q':
                print("\n\nQuitting...")
                break
            elif key == 'h' or key == '?':
                print_help()
            elif key == 'w':
                speed = min(1.0, speed + speed_step)
                node.set_control(speed, angle)
            elif key == 's':
                speed = max(-1.0, speed - speed_step)
                node.set_control(speed, angle)
            elif key == 'a':
                angle = max(-45.0, angle - angle_step)
                node.set_control(speed, angle)
            elif key == 'd':
                angle = min(45.0, angle + angle_step)
                node.set_control(speed, angle)
            elif key == 'm':
                new_mode = SystemMode.MANUAL if node.system_mode == SystemMode.AUTO else SystemMode.AUTO
                node.set_mode(new_mode)
            elif key == 'r':
                node.arm()
            elif key == 'u':
                node.disarm()
            elif key == 'l':
                lights_on = not lights_on
                node.set_lights(lights_on)
            elif key == 'e':
                node.emergency_brake()
                print("\n\n*** EMERGENCY BRAKE ACTIVATED ***")
            
            print_status(node, speed, angle, node.system_mode, node.armed, lights_on)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Interactive CLI for Jetson control node testing'
    )
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyUSB0',
        help='Serial port path (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=921600,
        help='Baud rate (default: 921600)'
    )
    parser.add_argument(
        '--control-hz',
        type=float,
        default=100.0,
        help='Control update frequency in Hz (default: 100.0)'
    )
    parser.add_argument(
        '--heartbeat-hz',
        type=float,
        default=10.0,
        help='Heartbeat frequency in Hz (default: 10.0)'
    )
    
    args = parser.parse_args()
    
    # Create control node
    node = JetsonControlNode(
        port=args.port,
        baudrate=args.baudrate,
        control_hz=args.control_hz,
        heartbeat_hz=args.heartbeat_hz
    )
    
    try:
        # Start node
        print(f"Connecting to {args.port} at {args.baudrate} bps...")
        node.start()
        
        if not node.running:
            print("Failed to start control node. Check serial port connection.")
            return 1
        
        print("Control node started. Starting interactive mode...")
        time.sleep(0.5)  # Give threads time to initialize
        
        # Initial setup
        node.set_mode(SystemMode.MANUAL)  # Start in manual mode for safety
        node.disarm()  # Start disarmed
        
        # Run interactive control
        interactive_control(node)
        
        # Final stats
        print("\n\nFinal Statistics:")
        stats = node.get_stats()
        for key, value in stats.items():
            print(f"  {key}: {value}")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 1
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        node.stop_all()


if __name__ == '__main__':
    sys.exit(main())

