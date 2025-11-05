# Jetson Brain Control Node for ESP32 Communication

High-speed UART control module for Jetson to ESP32 communication in autonomous vehicle systems.

## Overview

This module implements a binary protocol for real-time control communication between a Jetson (brain) and ESP32 (executor). The Jetson handles perception and decision-making, while the ESP32 executes motor control, steering, and other hardware operations using FreeRTOS.

## Protocol Specification

### Message Format

Each message is **16 bytes** total:

```
Byte 0:     Start byte (0xAA)
Byte 1:     Topic (1-4)
Byte 2:     Command ID
Bytes 3-6:  Value (int32_t, little-endian)
Bytes 7-8:  Sequence (uint16_t, little-endian)
Bytes 9-10: TTL (uint16_t, milliseconds, little-endian)
Bytes 11-12: (reserved/unused)
Bytes 13-14: CRC16 (little-endian)
```

### Topics

- `T_DRIVE = 1`: Drive/motor commands
- `T_STEER = 2`: Steering commands
- `T_LIGHTS = 3`: Light control
- `T_SYS = 4`: System management

### Commands

#### Drive Commands (Topic 1)
- `DRIVE_SET_SPEED = 1`: Set speed setpoint
- `DRIVE_EBRAKE = 2`: Emergency brake
- `DRIVE_STOP = 3`: Stop command

#### Steering Commands (Topic 2)
- `STEER_SET_ANGLE = 10`: Set steering angle

#### Lights Commands (Topic 3)
- `LIGHTS_ON = 30`: Turn lights on
- `LIGHTS_OFF = 31`: Turn lights off

#### System Commands (Topic 4)
- `SYS_HEARTBEAT = 20`: Heartbeat message
- `SYS_MODE = 21`: Set mode (0=AUTO, 1=MANUAL)
- `SYS_ARM = 22`: Arm the system
- `SYS_DISARM = 23`: Disarm the system

## Installation

```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

```python
from jetson_control import JetsonControlNode, SystemMode

# Create control node
node = JetsonControlNode(
    port='/dev/ttyUSB0',
    baudrate=921600,
    control_hz=100.0,      # 100 Hz control updates
    heartbeat_hz=10.0      # 10 Hz heartbeat
)

# Start communication
node.start()

# Set system mode and arm
node.set_mode(SystemMode.AUTO)
node.arm()

# Update control setpoints (these are sent at 100 Hz automatically)
node.set_control(speed=0.5, steer_angle=10.0)

# Emergency brake (sent immediately)
node.emergency_brake()

# Clean shutdown
node.stop_all()
```

### Interactive CLI Testing

Use the interactive CLI for manual testing:

```bash
python test_cli.py --port /dev/ttyUSB0 --baudrate 921600
```

**Controls:**
- `W/S`: Increase/Decrease speed
- `A/D`: Increase/Decrease steering angle
- `M`: Toggle AUTO/MANUAL mode
- `R`: Arm system
- `U`: Disarm system
- `L`: Toggle lights
- `E`: Emergency brake
- `Q`: Quit
- `H/?`: Show help

### Integration with AI/Planner

```python
import cv2
from jetson_control import JetsonControlNode, SystemMode

node = JetsonControlNode()
node.start()
node.set_mode(SystemMode.AUTO)
node.arm()

# Main perception/planning loop
while True:
    # Your perception code (e.g., YOLO, OpenCV)
    frame = camera.read()
    detections = yolo_model(frame)
    
    # Your planning code
    speed, angle = planner.compute_control(detections)
    
    # Send control commands
    node.set_control(speed=speed, steer_angle=angle)
    
    # Check for emergency conditions
    if emergency_detected:
        node.emergency_brake()
```

## Architecture

### Threading Model

The control node uses multiple worker threads:

1. **Send Thread**: Processes message queue and sends over UART
2. **Receive Thread**: Receives and validates incoming messages
3. **Control Thread**: Sends control updates at 100 Hz
4. **Heartbeat Thread**: Sends heartbeat at 10 Hz

### Message Queue

All messages are queued and sent asynchronously to prevent blocking. Emergency messages can be sent immediately via priority handling.

### Statistics

The node tracks:
- Sent/received message counts
- CRC errors
- Round-trip latency (if ACK messages are implemented)
- Queue size

Access via `node.get_stats()`.

## Message Timing

- **Control Updates**: 100 Hz (every 10 ms)
- **Heartbeat**: 10 Hz (every 100 ms)
- **Emergency**: Immediate (priority)
- **TTL Values**:
  - Control: 100 ms
  - Heartbeat: 200 ms
  - Emergency: 80 ms

## CRC16 Calculation

The protocol uses CRC16-XMODEM (polynomial 0x1021) calculated over the 13-byte payload (topic through TTL).

## Error Handling

- Automatic CRC validation on received messages
- Serial port error recovery
- Queue overflow protection
- Thread-safe state management

## File Structure

```
brain/
├── protocol.py          # Protocol definitions, packing/unpacking, CRC16
├── jetson_control.py    # Main control node implementation
├── test_cli.py          # Interactive CLI for testing
├── requirements.txt     # Python dependencies
└── README.md           # This file
```

## Configuration

### Serial Port Settings

- **Baud Rate**: 921600 (default, configurable)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Timeout**: 100 ms (read), 1 s (write)

### Adjustable Parameters

- `control_hz`: Control update frequency (default: 100 Hz)
- `heartbeat_hz`: Heartbeat frequency (default: 10 Hz)
- `control_ttl`: TTL for control messages (default: 100 ms)
- `heartbeat_ttl`: TTL for heartbeat (default: 200 ms)
- `emergency_ttl`: TTL for emergency (default: 80 ms)

## Troubleshooting

### Serial Port Not Found

Check available ports:
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Ensure user has permissions:
```bash
sudo usermod -a -G dialout $USER
# Then logout/login
```

### High Latency

- Reduce `control_hz` if ESP32 cannot process at 100 Hz
- Check UART baud rate matches on both sides
- Monitor queue size via `get_stats()`

### CRC Errors

- Verify baud rate matches ESP32 configuration
- Check wiring/connections
- Ensure ESP32 is sending properly formatted messages

## Future Enhancements

- [ ] ACK message handling and round-trip latency measurement
- [ ] Telemetry visualization dashboard
- [ ] Message priority queue for emergency commands
- [ ] Auto-resend last control if no update received
- [ ] Configuration file support
- [ ] ROS2 integration wrapper

## License

Part of the BFMC Autonomous Laboratory Robot project.

