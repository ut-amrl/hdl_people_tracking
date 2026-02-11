#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import struct

class DummyMapPublisher(Node):
    def __init__(self):
        super().__init__('dummy_map_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/globalmap', 10)
        # Publish at 1Hz to ensure it is received even if subscriber connects late
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Dummy Map Publisher Started. Frame: odom')

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        msg.height = 1
        msg.width = 1
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16
        msg.is_dense = True
        
        # Single point at 0,0,0 with intensity 1.0
        msg.data = struct.pack('ffff', 0.0, 0.0, 0.0, 1.0)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
