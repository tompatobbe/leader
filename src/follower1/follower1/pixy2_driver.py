#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import spidev
import struct
import time

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        
        self.spi_bus = 0
        self.spi_device = 0
        self.target_signature = 1  
        
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000 
            self.spi.mode = 0b00
            self.get_logger().info("SPI Initialized. Waiting for blocks...")
        except Exception as e:
            self.get_logger().error(f"SPI Error: {e}")
            return

        self.timer = self.create_timer(0.04, self.update)

    def update(self):
        # Request blocks: [Sync, Sync, Type=32, Len=2, Sig, MaxBlocks]
        req = [0xae, 0xc1, 0x20, 0x02, self.target_signature, 0x02]
        try:
            self.spi.xfer2(req)
        except OSError:
            return # Ignore momentary SPI busy errors

        # Hunt for Sync (0xaf, 0xc1)
        for _ in range(50):
            try:
                if self.spi.readbytes(1)[0] == 0xaf:
                    if self.spi.readbytes(1)[0] == 0xc1:
                        self.process_packet()
                        return
            except Exception:
                pass

    def process_packet(self):
        # Read Header: [Type, Length, ChecksumL, ChecksumH]
        header = self.spi.readbytes(4)
        packet_type = header[0]
        payload_len = header[1]
        
        if payload_len == 0 or packet_type != 0x21:
            return

        # Read the full payload
        payload = self.spi.readbytes(payload_len)
        self.parse_blocks(payload)

    def parse_blocks(self, payload):
        # We determine block size based on payload length
        # Standard Pixy2 block is 14 bytes. 
        # But if you see errors about 12 bytes, we can adapt.
        
        if len(payload) % 14 == 0:
            block_size = 14
        elif len(payload) % 12 == 0:
            block_size = 12
        else:
            # If payload is 20 bytes, that's weird. Just fallback to 14.
            block_size = 14

        num_blocks = len(payload) // block_size
        
        for i in range(num_blocks):
            start = i * block_size
            end = start + block_size
            block_data = bytearray(payload[start:end])

            try:
                # --- AUTO-DETECT FORMAT ---
                if len(block_data) == 14:
                    # Full format: Sig, X, Y, W, H, Angle, Index, Age
                    # < = Little Endian
                    # H = UShort(2), h = Short(2), B = Byte(1)
                    # Total: 2+2+2+2+2+2+1+1 = 14
                    sig, x, y, w, h, angle, idx, age = struct.unpack('<HHHHhBB', block_data)
                    self.get_logger().info(f"Target: X={x}, Y={y} | W={w}, H={h}")
                
                elif len(block_data) == 12:
                    # Short format (Missing Index/Age?): Sig, X, Y, W, H, Angle
                    # Total: 2+2+2+2+2+2 = 12
                    sig, x, y, w, h, angle = struct.unpack('<HHHHh', block_data)
                    self.get_logger().info(f"Target (Short): X={x}, Y={y} | W={w}, H={h}")

            except struct.error as e:
                # This prints exactly what mismatched so we can debug
                self.get_logger().warn(f"Struct Error: {e} | Buffer Size: {len(block_data)}")

def main(args=None):
    rclpy.init(args=args)
    node = Pixy2SpiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()