#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import spidev
import struct
import time

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        
        # --- CONFIGURATION ---
        self.spi_bus = 0
        self.spi_device = 0
        self.target_signature = 1  # 1 = Sig 1, 255 = All
        
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000 
            self.spi.mode = 0b00
            self.get_logger().info("SPI Connected. Starting Stream...")
        except Exception as e:
            self.get_logger().error(f"SPI Fail: {e}")
            return

        # Poll rapidly
        self.timer = self.create_timer(0.04, self.update)

    def update(self):
        # 1. Send "Get Blocks" Request
        # [Sync(2), Type(1), Length(1), Sig(1), MaxBlocks(1)]
        # Note: 0x20 is command "getBlocks", 0x02 is data length
        req = [0xae, 0xc1, 0x20, 0x02, self.target_signature, 0x02]
        self.spi.xfer2(req)

        # 2. SYNC: Read bytes one by one until we find 0xAF 0xC1
        # (We limit this loop to avoid hanging forever)
        for _ in range(50):
            byte = self.spi.readbytes(1)[0]
            if byte == 0xaf:
                # Potential sync, check next byte
                byte2 = self.spi.readbytes(1)[0]
                if byte2 == 0xc1:
                    self.process_packet()
                    return # Done for this cycle

    def process_packet(self):
        # We found Sync (0xAF 0xC1). Now read the Header (4 bytes)
        # Header: [Type, Length, ChecksumL, ChecksumH]
        header = self.spi.readbytes(4)
        
        packet_type = header[0]
        payload_len = header[1]
        checksum = struct.unpack('<H', bytearray(header[2:4]))[0]
        
        # If no data (len=0), we are done
        if payload_len == 0:
            return

        # 3. Read the Payload
        # We now know EXACTLY how many bytes to read
        payload = self.spi.readbytes(payload_len)
        
        # Validate Packet Type (0x21 is the response to getBlocks)
        if packet_type == 0x21:
            self.parse_blocks(payload)
        else:
            # If it's not a block response, ignore it
            pass

    def parse_blocks(self, payload):
        # A block is 14 bytes
        BLOCK_SIZE = 14
        num_blocks = len(payload) // BLOCK_SIZE
        
        for i in range(num_blocks):
            start = i * BLOCK_SIZE
            end = start + BLOCK_SIZE
            
            # Extract the slice for this block
            block_data = bytearray(payload[start:end])
            
            # SAFETY: Ensure we actually have 14 bytes
            if len(block_data) != BLOCK_SIZE:
                continue

            try:
                # Bytes:
                # 0-1: Sig, 2-3: X, 4-5: Y, 6-7: W, 8-9: H
                # 10-11: Angle, 12: Index, 13: Age
                sig, x, y, w, h, angle, idx, age = struct.unpack('<HHHHhBB', block_data)
                
                # --- VISUALIZATION ---
                # Print coordinates to terminal
                self.get_logger().info(f"Object {i+1}: [Sig {sig}] X:{x} Y:{y} (W:{w} H:{h})")
                
            except struct.error as e:
                self.get_logger().warn(f"Struct Error: {e} | Data Len: {len(block_data)}")

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