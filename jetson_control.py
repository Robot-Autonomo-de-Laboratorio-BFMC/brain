#!/usr/bin/env python3
"""
Jetson Brain Control Node for ESP32 Communication.

Handles high-speed UART communication with ESP32 executor.
Sends control commands, heartbeat, and emergency signals.
"""

import serial
import threading
import time
import queue
import logging
from typing import Optional, Tuple
from enum import IntEnum
from dataclasses import dataclass

from protocol import (
    pack_message, unpack_message, message_to_string,
    Topic, DriveCmd, SteerCmd, LightsCmd, SysCmd, SystemMode,
    START_BYTE, MESSAGE_SIZE
)


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ControlState:
    """Current control state (speed, steering angle)."""
    speed: float = 0.0  # m/s or normalized [-1.0, 1.0]
    steer_angle: float = 0.0  # degrees or normalized [-1.0, 1.0]
    last_update: float = 0.0  # timestamp


@dataclass
class MessageStats:
    """Statistics for sent/received messages."""
    sent_count: int = 0
    received_count: int = 0
    crc_errors: int = 0
    last_sent_time: float = 0.0
    last_received_time: float = 0.0
    round_trip_times: list = None
    
    def __post_init__(self):
        if self.round_trip_times is None:
            self.round_trip_times = []


class JetsonControlNode:
    """
    Main control node for Jetson-ESP32 communication.
    
    Features:
    - High-speed UART communication (921600 bps)
    - Periodic control updates (100 Hz)
    - Heartbeat (10 Hz)
    - Emergency brake handling
    - Message statistics and logging
    """
    
    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 921600,
        control_hz: float = 100.0,
        heartbeat_hz: float = 10.0,
        control_ttl: int = 100,
        heartbeat_ttl: int = 200,
        emergency_ttl: int = 80
    ):
        """
        Initialize the control node.
        
        Args:
            port: UART device path (e.g., '/dev/ttyUSB0')
            baudrate: UART baud rate (default: 921600)
            control_hz: Control update frequency (default: 100 Hz)
            heartbeat_hz: Heartbeat frequency (default: 10 Hz)
            control_ttl: TTL for control messages (ms)
            heartbeat_ttl: TTL for heartbeat messages (ms)
            emergency_ttl: TTL for emergency messages (ms)
        """
        self.port = port
        self.baudrate = baudrate
        self.control_hz = control_hz
        self.heartbeat_hz = heartbeat_hz
        self.control_ttl = control_ttl
        self.heartbeat_ttl = heartbeat_ttl
        self.emergency_ttl = emergency_ttl
        
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.sequence = 0
        
        # Control state
        self.control_state = ControlState()
        self.system_mode = SystemMode.AUTO
        self.armed = False
        
        # Message queue for async sending
        self.message_queue = queue.Queue()
        
        # Statistics
        self.stats = MessageStats()
        
        # Threading
        self.send_thread: Optional[threading.Thread] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.control_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        
        # Locks
        self.state_lock = threading.Lock()
        self.stats_lock = threading.Lock()
    
    def _get_next_sequence(self) -> int:
        """Get next sequence number (thread-safe)."""
        seq = self.sequence
        self.sequence = (self.sequence + 1) % 65536
        return seq
    
    def connect(self) -> bool:
        """
        Open UART connection.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 100ms timeout for reads
                write_timeout=1.0
            )
            logger.info(f"Connected to {self.port} at {self.baudrate} bps")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close UART connection."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("Disconnected from serial port")
    
    def send_message(self, topic: int, command: int, value: int, ttl: int) -> bool:
        """
        Send a message to ESP32.
        
        Args:
            topic: Message topic
            command: Command ID
            value: 32-bit value
            ttl: Time-to-live in milliseconds
            
        Returns:
            True if message queued successfully
        """
        seq = self._get_next_sequence()
        message = pack_message(topic, command, value, seq, ttl)
        
        try:
            self.message_queue.put((message, topic, command, value, seq, ttl), timeout=0.1)
            with self.stats_lock:
                self.stats.sent_count += 1
                self.stats.last_sent_time = time.time()
            return True
        except queue.Full:
            logger.warning("Message queue full, dropping message")
            return False
    
    def _send_worker(self):
        """Worker thread that sends messages from queue."""
        while self.running:
            try:
                message, topic, command, value, seq, ttl = self.message_queue.get(timeout=0.1)
                
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(message)
                        self.serial_port.flush()
                        logger.debug(
                            f"Sent: {message_to_string(topic, command, value, seq, ttl)}"
                        )
                    except serial.SerialTimeoutException:
                        logger.error("Serial write timeout")
                    except serial.SerialException as e:
                        logger.error(f"Serial write error: {e}")
                
                self.message_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in send worker: {e}")
    
    def _receive_worker(self):
        """Worker thread that receives and processes messages from ESP32."""
        buffer = bytearray()
        
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.01)
                continue
            
            try:
                # Read available data
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if not data:
                    time.sleep(0.001)
                    continue
                
                buffer.extend(data)
                
                # Process complete messages
                while len(buffer) >= MESSAGE_SIZE:
                    # Look for start byte
                    start_idx = buffer.find(START_BYTE)
                    if start_idx == -1:
                        buffer.clear()
                        break
                    
                    if start_idx > 0:
                        buffer = buffer[start_idx:]
                    
                    if len(buffer) < MESSAGE_SIZE:
                        break
                    
                    # Extract message
                    message_data = bytes(buffer[:MESSAGE_SIZE])
                    buffer = buffer[MESSAGE_SIZE:]
                    
                    # Unpack and validate
                    result = unpack_message(message_data)
                    if result:
                        topic, command, value, seq, ttl, crc = result
                        with self.stats_lock:
                            self.stats.received_count += 1
                            self.stats.last_received_time = time.time()
                        
                        logger.debug(
                            f"Received: {message_to_string(topic, command, value, seq, ttl)}"
                        )
                        # Handle ACK or status messages here if needed
                    else:
                        with self.stats_lock:
                            self.stats.crc_errors += 1
                        logger.warning("Invalid message received (CRC error or malformed)")
                        
            except serial.SerialException as e:
                logger.error(f"Serial read error: {e}")
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Error in receive worker: {e}")
                time.sleep(0.1)
    
    def _control_worker(self):
        """Worker thread that sends control updates at specified frequency."""
        period = 1.0 / self.control_hz
        
        while self.running:
            start_time = time.time()
            
            with self.state_lock:
                speed = self.control_state.speed
                steer_angle = self.control_state.steer_angle
            
            # Send speed command
            if abs(speed) > 0.001 or True:  # Always send for now
                speed_value = int(speed * 1000)  # Convert to mm/s or scaled int
                self.send_message(
                    Topic.T_DRIVE,
                    DriveCmd.DRIVE_SET_SPEED,
                    speed_value,
                    self.control_ttl
                )
            
            # Send steering command
            if abs(steer_angle) > 0.001 or True:  # Always send for now
                angle_value = int(steer_angle * 100)  # Convert to 0.01 degree units
                self.send_message(
                    Topic.T_STEER,
                    SteerCmd.STEER_SET_ANGLE,
                    angle_value,
                    self.control_ttl
                )
            
            # Sleep to maintain frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _heartbeat_worker(self):
        """Worker thread that sends heartbeat at specified frequency."""
        period = 1.0 / self.heartbeat_hz
        
        while self.running:
            self.send_message(
                Topic.T_SYS,
                SysCmd.SYS_HEARTBEAT,
                0,
                self.heartbeat_ttl
            )
            time.sleep(period)
    
    def set_control(self, speed: float, steer_angle: float):
        """
        Update control setpoints.
        
        Args:
            speed: Speed setpoint (m/s or normalized)
            steer_angle: Steering angle setpoint (degrees or normalized)
        """
        with self.state_lock:
            self.control_state.speed = speed
            self.control_state.steer_angle = steer_angle
            self.control_state.last_update = time.time()
    
    def emergency_brake(self):
        """Send emergency brake command immediately."""
        logger.warning("EMERGENCY BRAKE triggered")
        self.send_message(
            Topic.T_DRIVE,
            DriveCmd.DRIVE_EBRAKE,
            0,
            self.emergency_ttl
        )
    
    def stop(self):
        """Send stop command."""
        self.send_message(
            Topic.T_DRIVE,
            DriveCmd.DRIVE_STOP,
            0,
            self.control_ttl
        )
    
    def set_mode(self, mode: SystemMode):
        """
        Set system mode (AUTO/MANUAL).
        
        Args:
            mode: SystemMode.AUTO or SystemMode.MANUAL
        """
        self.system_mode = mode
        self.send_message(
            Topic.T_SYS,
            SysCmd.SYS_MODE,
            int(mode),
            self.control_ttl
        )
        logger.info(f"System mode set to: {mode.name}")
    
    def arm(self):
        """Arm the system."""
        self.armed = True
        self.send_message(
            Topic.T_SYS,
            SysCmd.SYS_ARM,
            0,
            self.control_ttl
        )
        logger.info("System ARMED")
    
    def disarm(self):
        """Disarm the system."""
        self.armed = False
        self.send_message(
            Topic.T_SYS,
            SysCmd.SYS_DISARM,
            0,
            self.control_ttl
        )
        logger.info("System DISARMED")
    
    def set_lights(self, on: bool):
        """
        Control lights.
        
        Args:
            on: True to turn on, False to turn off
        """
        cmd = LightsCmd.LIGHTS_ON if on else LightsCmd.LIGHTS_OFF
        self.send_message(
            Topic.T_LIGHTS,
            cmd,
            0,
            self.control_ttl
        )
    
    def start(self):
        """Start all worker threads."""
        if self.running:
            logger.warning("Control node already running")
            return
        
        if not self.serial_port or not self.serial_port.is_open:
            if not self.connect():
                logger.error("Failed to connect, cannot start")
                return
        
        self.running = True
        
        # Start worker threads
        self.send_thread = threading.Thread(target=self._send_worker, daemon=True)
        self.receive_thread = threading.Thread(target=self._receive_worker, daemon=True)
        self.control_thread = threading.Thread(target=self._control_worker, daemon=True)
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_worker, daemon=True)
        
        self.send_thread.start()
        self.receive_thread.start()
        self.control_thread.start()
        self.heartbeat_thread.start()
        
        logger.info("Jetson control node started")
    
    def stop_all(self):
        """Stop all worker threads and close connection."""
        logger.info("Stopping Jetson control node...")
        self.running = False
        
        # Wait for threads to finish
        if self.send_thread:
            self.send_thread.join(timeout=2.0)
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=2.0)
        
        self.disconnect()
        logger.info("Jetson control node stopped")
    
    def get_stats(self) -> dict:
        """Get message statistics."""
        with self.stats_lock:
            return {
                'sent_count': self.stats.sent_count,
                'received_count': self.stats.received_count,
                'crc_errors': self.stats.crc_errors,
                'last_sent_time': self.stats.last_sent_time,
                'last_received_time': self.stats.last_received_time,
                'queue_size': self.message_queue.qsize()
            }


def main():
    """Example usage."""
    node = JetsonControlNode(
        port='/dev/ttyUSB0',
        baudrate=921600,
        control_hz=100.0,
        heartbeat_hz=10.0
    )
    
    try:
        node.start()
        
        # Set initial mode
        node.set_mode(SystemMode.AUTO)
        node.arm()
        
        # Example: set some control values
        node.set_control(speed=0.5, steer_angle=10.0)
        
        # Run for a while
        time.sleep(5.0)
        
        # Print stats
        stats = node.get_stats()
        logger.info(f"Statistics: {stats}")
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        node.stop_all()


if __name__ == '__main__':
    main()

