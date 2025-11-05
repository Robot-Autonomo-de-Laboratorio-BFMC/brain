#!/usr/bin/env python3
"""
Quick test script to verify protocol packing/unpacking.
"""

from protocol import (
    pack_message, unpack_message, message_to_string,
    Topic, DriveCmd, SysCmd, MESSAGE_SIZE
)


def test_pack_unpack():
    """Test packing and unpacking messages."""
    print("Testing protocol packing/unpacking...")
    
    # Test cases
    test_cases = [
        (Topic.T_DRIVE, DriveCmd.DRIVE_SET_SPEED, 500, 1, 100),
        (Topic.T_STEER, 10, 1000, 2, 100),
        (Topic.T_SYS, SysCmd.SYS_HEARTBEAT, 0, 3, 200),
        (Topic.T_SYS, SysCmd.SYS_MODE, 0, 4, 100),  # AUTO mode
        (Topic.T_DRIVE, DriveCmd.DRIVE_EBRAKE, 0, 5, 80),
    ]
    
    for topic, cmd, value, seq, ttl in test_cases:
        # Pack message
        msg = pack_message(topic, cmd, value, seq, ttl)
        
        # Verify size
        assert len(msg) == MESSAGE_SIZE, f"Message size incorrect: {len(msg)} != {MESSAGE_SIZE}"
        
        # Verify start byte
        assert msg[0] == 0xAA, f"Start byte incorrect: {msg[0]:02X} != 0xAA"
        
        # Unpack message
        result = unpack_message(msg)
        assert result is not None, "Failed to unpack message"
        
        topic_out, cmd_out, value_out, seq_out, ttl_out, crc_out = result
        
        # Verify values
        assert topic_out == topic, f"Topic mismatch: {topic_out} != {topic}"
        assert cmd_out == cmd, f"Command mismatch: {cmd_out} != {cmd}"
        assert value_out == value, f"Value mismatch: {value_out} != {value}"
        assert seq_out == seq, f"Sequence mismatch: {seq_out} != {seq}"
        assert ttl_out == ttl, f"TTL mismatch: {ttl_out} != {ttl}"
        
        print(f"✓ {message_to_string(topic, cmd, value, seq, ttl)}")
    
    print("\nAll tests passed!")
    
    # Print a sample message in hex
    sample = pack_message(Topic.T_DRIVE, DriveCmd.DRIVE_SET_SPEED, 500, 123, 100)
    print(f"\nSample message (hex):")
    print(' '.join(f'{b:02X}' for b in sample))
    print(f"Message size: {len(sample)} bytes")


if __name__ == '__main__':
    test_pack_unpack()

