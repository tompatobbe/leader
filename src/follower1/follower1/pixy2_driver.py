#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import spidev
import struct

class Pixy2SpiNode(Node):
    def __init__(self):
        super().__init__('pixy2_driver')
        self.publisher_ = self.create_publisher(Point, '/pixy/target', 10)
        
        # SPI Setup
        self.spi_bus = 0
        self.spi_device = 0
        self.spi = spidev.SpiDev()
        
        try:
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 2000000
            self.spi.mode = 0b00
            self.get_logger().info("Pixy2 Driver: SPI Connected. Scanning for blocks...")
        except Exception as e:
            self.get_logger().error(f"SPI Failed: {e}")
            return

        # Poll at 30Hz
        self.timer = self.create_timer(0.03, self.update)

    def update(self):
        # 1. Send Request (Ask for Sig 1)
        req = [0xae, 0xc1, 0x20, 0x02, 1, 0x02]
        try:
            self.spi.xfer2(req)
        except OSError:
            return

        # 2. Read a BIG chunk (64 bytes) to capture the response
        response = self.spi.readbytes(64)
        data_bytes = bytearray(response)

        # 3. Find the Sync Word (0xaf 0xc1)
        try:
            idx = data_bytes.find(b'\xaf\xc1')
            
            if idx != -1:
                # Safety check: do we have enough bytes left for a header?
                if idx + 6 > len(data_bytes):
                    return

                pkt_type = data_bytes[idx + 2]
                payload_len = data_bytes[idx + 3]
                
                # Check if it's the right packet (GetBlocks Response = 0x21)
                if pkt_type == 0x21 and payload_len > 0:
                    payload_start = idx + 6
                    
                    if payload_start + payload_len <= len(data_bytes):
                        payload = data_bytes[payload_start : payload_start + payload_len]
                        self.parse_block(payload)

        except Exception as e:
            self.get_logger().warn(f"Scan Error: {e}")

    def parse_block(self, payload):
        # We need at least 12 bytes for the coordinates (Sig, X, Y, W, H, Angle)
        if len(payload) >= 12:
            try:
                # --- THE FIX IS HERE ---
                # < = Little Endian
                # H = Unsigned Short (2 bytes)
                # h = Signed Short (2 bytes)
                # Format: H(Sig) H(X) H(Y) H(W) H(H) h(Angle) -> 6 items
                data = struct.unpack_from('<HHHHHh', payload, offset=0)
                
                sig, x, y, w, h, angle = data
                
                # Publish
                msg = Point()
                msg.x = float(x)
                msg.y = float(y)
                msg.z = float(w) # Using Z for Width
                self.publisher_.publish(msg)

                # Print to terminal
                self.get_logger().info(f"Target Found! X={x}, Y={y}, Width={w}")
                
            except struct.error as e:
                self.get_logger().warn(f"Unpack Error: {e}")

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