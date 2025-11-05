"""
Binary protocol for Jetson-ESP32 communication.

Message format:
- 1 byte: Start (0xAA)
- 1 byte: Topic
- 1 byte: Command
- 4 bytes: Value (int32_t, little-endian)
- 2 bytes: Sequence (uint16_t, little-endian)
- 2 bytes: TTL (uint16_t, little-endian, milliseconds)
- 2 bytes: CRC16 (little-endian)
Total: 16 bytes
"""

import struct
from enum import IntEnum
from typing import Tuple, Optional


# Topics
class Topic(IntEnum):
    T_DRIVE = 1
    T_STEER = 2
    T_LIGHTS = 3
    T_SYS = 4


# Commands
class DriveCmd(IntEnum):
    DRIVE_SET_SPEED = 1
    DRIVE_EBRAKE = 2
    DRIVE_STOP = 3


class SteerCmd(IntEnum):
    STEER_SET_ANGLE = 10


class LightsCmd(IntEnum):
    LIGHTS_ON = 30
    LIGHTS_OFF = 31


class SysCmd(IntEnum):
    SYS_HEARTBEAT = 20
    SYS_MODE = 21  # value = 0 for AUTO, 1 for MANUAL
    SYS_ARM = 22
    SYS_DISARM = 23


# System modes
class SystemMode(IntEnum):
    AUTO = 0
    MANUAL = 1


# Protocol constants
START_BYTE = 0xAA
MESSAGE_SIZE = 16
PAYLOAD_SIZE = 13


def crc16_xmodem(data: bytes) -> int:
    """
    Calculate CRC16 using XMODEM polynomial (0x1021).
    
    Args:
        data: Input bytes to calculate CRC for
        
    Returns:
        16-bit CRC value
    """
    crc = 0x0000
    polynomial = 0x1021
    
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFFFF
    
    return crc


def pack_message(topic: int, command: int, value: int, sequence: int, ttl: int) -> bytes:
    """
    Pack a message into binary format.
    
    Args:
        topic: Message topic (1-4)
        command: Command ID
        value: 32-bit signed integer value
        sequence: 16-bit sequence number
        ttl: Time-to-live in milliseconds (16-bit)
        
    Returns:
        16-byte message packet
    """
    # Pack payload (topic + command + value + sequence + ttl + 3 reserved bytes)
    # Total payload: 1 + 1 + 4 + 2 + 2 + 3 = 13 bytes
    payload = struct.pack(
        '<BBiHH3x',  # little-endian: u8, u8, i32, u16, u16, 3 padding bytes
        topic,
        command,
        value,
        sequence & 0xFFFF,
        ttl & 0xFFFF
    )
    
    # Calculate CRC16 over payload (13 bytes)
    crc = crc16_xmodem(payload)
    
    # Pack full message: start byte + payload + CRC
    message = struct.pack(
        '<B13sH',  # u8 + 13 bytes payload + u16 CRC
        START_BYTE,
        payload,
        crc & 0xFFFF
    )
    
    return message


def unpack_message(data: bytes) -> Optional[Tuple[int, int, int, int, int, int]]:
    """
    Unpack a message from binary format.
    
    Args:
        data: 16-byte message packet
        
    Returns:
        Tuple of (topic, command, value, sequence, ttl, crc) or None if invalid
    """
    if len(data) != MESSAGE_SIZE:
        return None
    
    if data[0] != START_BYTE:
        return None
    
    # Extract payload (13 bytes: topic + cmd + value + seq + ttl + 3 reserved)
    payload = data[1:14]
    
    # Unpack payload (skip the 3 reserved bytes at the end)
    topic, command, value, sequence, ttl = struct.unpack('<BBiHH3x', payload)
    
    # Unpack CRC
    received_crc, = struct.unpack('<H', data[14:16])
    
    # Verify CRC over payload
    calculated_crc = crc16_xmodem(payload)
    
    if received_crc != calculated_crc:
        return None
    
    return (topic, command, value, sequence, ttl, received_crc)


def message_to_string(topic: int, command: int, value: int, sequence: int, ttl: int) -> str:
    """Generate a human-readable string representation of a message."""
    topic_str = {1: "DRIVE", 2: "STEER", 3: "LIGHTS", 4: "SYS"}.get(topic, f"TOPIC_{topic}")
    
    cmd_map = {
        1: "SET_SPEED", 2: "EBRAKE", 3: "STOP",
        10: "SET_ANGLE",
        20: "HEARTBEAT", 21: "MODE", 22: "ARM", 23: "DISARM",
        30: "LIGHTS_ON", 31: "LIGHTS_OFF"
    }
    cmd_str = cmd_map.get(command, f"CMD_{command}")
    
    if topic == Topic.T_SYS and command == SysCmd.SYS_MODE:
        mode_str = "AUTO" if value == 0 else "MANUAL"
        return f"{topic_str}.{cmd_str}({mode_str}) seq={sequence} ttl={ttl}ms"
    
    return f"{topic_str}.{cmd_str}({value}) seq={sequence} ttl={ttl}ms"

