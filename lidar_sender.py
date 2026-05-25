import serial
import struct
import time

# -----------------------------------------------
# D500 LiDAR connection (USB)
# Run: ls /dev/ttyUSB* on your Pi terminal
# to find the correct port, then update below
# -----------------------------------------------
lidar_serial = serial.Serial('/dev/ttyUSB0', baudrate=230400, timeout=1)

# -----------------------------------------------
# Arduino connection (GPIO Serial)
# Pi GPIO 14 (TX) --> Arduino pin 19 (RX1)
# Pi GPIO 15 (RX) --> Arduino pin 18 (TX1)
# Both GND must be connected together
# -----------------------------------------------
arduino_serial = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)

HEADER     = 0x54
PACKET_LEN = 47
NUM_POINTS = 12
FRONT_ZONE = 30   # degrees either side of 0° counted as front

# CRC8 lookup table for D500 packet verification
CRC_TABLE = [
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,
    0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,
    0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,
    0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,
    0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
    0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,
    0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
    0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,
    0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,
    0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
    0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,
    0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
    0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,
    0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,
    0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
    0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,
    0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
    0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,
    0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,
    0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
    0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,
    0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
    0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,
    0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,
    0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
]


def calc_crc(data):
    """Calculate CRC8 checksum for packet verification."""
    crc = 0x00
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def parse_packet(buf):
    """
    Parse a 47-byte D500 packet.
    Returns the closest front-facing distance in cm, or None if no valid front point.
    """
    # Verify CRC — covers bytes 0 to 45, result must match byte 46
    if calc_crc(buf[:46]) != buf[46]:
        return None  # corrupted packet, discard

    # Extract start and end angles (little-endian uint16, unit: 0.01 degrees)
    start_angle = struct.unpack('<H', bytes(buf[4:6]))[0] * 0.01
    end_angle   = struct.unpack('<H', bytes(buf[42:44]))[0] * 0.01

    # Calculate angle step between the 12 measurement points
    if end_angle >= start_angle:
        angle_step = (end_angle - start_angle) / (NUM_POINTS - 1)
    else:
        # Handle wraparound (e.g. 355 degrees to 5 degrees)
        angle_step = (end_angle + 360.0 - start_angle) / (NUM_POINTS - 1)

    min_front_dist = 9999.0

    for i in range(NUM_POINTS):
        base = 6 + i * 3  # each point is 3 bytes: 2 bytes distance + 1 byte intensity

        dist_mm = struct.unpack('<H', bytes(buf[base:base + 2]))[0]

        if dist_mm == 0:
            continue  # 0 means invalid or out of range reading

        # Calculate the angle of this point
        angle = start_angle + i * angle_step
        if angle >= 360.0:
            angle -= 360.0

        # Check if this point is in the front zone (within FRONT_ZONE degrees of 0)
        is_front = (angle <= FRONT_ZONE) or (angle >= (360.0 - FRONT_ZONE))

        if is_front:
            dist_cm = dist_mm / 10.0
            if dist_cm < min_front_dist:
                min_front_dist = dist_cm

    return min_front_dist if min_front_dist < 9999.0 else None


def read_packet():
    """
    Read one complete 47-byte packet from the D500.
    Waits for the 0x54 header byte then reads the remaining 46 bytes.
    """
    while True:
        byte = lidar_serial.read(1)
        if not byte:
            continue
        if byte[0] == HEADER:
            rest = lidar_serial.read(PACKET_LEN - 1)
            if len(rest) == PACKET_LEN - 1:
                return [byte[0]] + list(rest)


def main():
    print("=" * 40)
    print("  D500 LiDAR → Arduino Distance Sender")
    print("=" * 40)
    print("Waiting for D500 to spin up (3 seconds)...")
    time.sleep(3)
    print("LiDAR ready. Sending front distances to Arduino.")
    print("-" * 40)

    while True:
        try:
            packet = read_packet()
            if packet:
                distance = parse_packet(packet)
                if distance:
                    # Send to Arduino as "D<value>\n"
                    # Example: "D85.3\n"
                    message = "D{:.1f}\n".format(distance)
                    arduino_serial.write(message.encode())
                    print("Sent to Arduino: {}".format(message.strip()))

            time.sleep(0.05)  # 20Hz send rate

        except serial.SerialException as e:
            print("Serial error: {}".format(e))
            time.sleep(1)

        except KeyboardInterrupt:
            print("\nStopped by user.")
            lidar_serial.close()
            arduino_serial.close()
            break


if __name__ == '__main__':
    main()
