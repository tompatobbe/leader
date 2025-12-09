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
        self.target_signature = 1  # Look for Signature 1 by default
        
        # Initialize SPI
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000 # 2 MHz is reliable for Pixy2
            self.spi.mode = 0b00
            self.get_logger().info(f"SPI Connected. Polling for objects...")
        except Exception as e:
            self.get_logger().error(f"Failed to open SPI: {e}")
            return

        # Poll at 20Hz (approx every 0.05s)
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        # --- STEP 1: Send Request ---
        # 0xae, 0xc1 = Sync (no checksum)
        # 0x20 = Type 32 (getBlocks)
        # 0x02 = Data Length (2 bytes follow)
        # 0x01 = Signature 1 (or 255 for all)
        # 0x02 = Max blocks to return 
        request = [0xae, 0xc1, 0x20, 0x02, self.target_signature, 0x02]
        self.spi.xfer2(request)

        # --- STEP 2: Read Header ---
        # INCREASED BUFFER: Reading 64 bytes gives us plenty of room 
        # for the header (6 bytes) + multiple blocks (14 bytes each)
        # + some padding/latency bytes.
        response = self.spi.readbytes(64)
        
        # --- STEP 3: Parse Response ---
        frame_start = -1
        # Scan for the Pixy2 response sync word: 0xAF, 0xC1
        for i in range(len(response) - 1):
            if response[i] == 0xaf and response[i+1] == 0xc1:
                frame_start = i
                break
        
        if frame_start == -1:
            return # No sync found

        # Decode the Header
        try:
            # We need at least 6 bytes from start to have a header
            if frame_start + 6 > len(response):
                return 

            data_type = response[frame_start + 2]
            length = response[frame_start + 3]
            
            # Type 33 (0x21) is the response to getBlocks
            if data_type == 0x21 and length > 0:
                payload_idx = frame_start + 6
                num_blocks = length // 14
                
                if num_blocks > 0:
                    self.parse_blocks(response, payload_idx, num_blocks)
                    
        except IndexError:
            pass 

    def parse_blocks(self, buffer, start_index, count):
        current_idx = start_index
        
        for i in range(count):
            # --- FIX: SAFETY CHECK ---
            # Ensure we actually have 14 bytes left in the buffer
            if current_idx + 14 > len(buffer):
                # We ran out of data, stop parsing to avoid the Error
                break
                
            block_bytes = bytearray(buffer[current_idx : current_idx+14])
            
            try:
                # Format: H(Sig), H(X), H(Y), H(W), H(H), h(Angle), B(Index), B(Age)
                sig, x, y, w, h, angle, idx, age = struct.unpack('<HHHHhBB', block_bytes)
                
                # Print clearly to terminal
                self.get_logger().info(f"Target Detected: X={x}, Y={y} (W={w}, H={h})")
                
            except struct.error as e:
                self.get_logger().warn(f"Packet parsing error: {e}")

            current_idx += 14
            
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