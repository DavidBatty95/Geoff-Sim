import time

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from std_msgs.msg import String

from unitree_g1_mujoco_bridge.dds_metadata_executor import DDSMetadataExecutor


def test_humble_dds_metadata_and_ordinary_callbacks_share_executor():
    context = Context()
    rclpy.init(context=context)
    node = Node('metadata_executor_test',context=context)
    received,ordinary = [],[]
    def callback(message,info):
        received.append((message.data,info))
    executor = DDSMetadataExecutor(metadata_callbacks=(callback,),context=context)
    executor.add_node(node)
    node.create_subscription(String,'metadata_probe',callback,1)
    node.create_subscription(String,'metadata_probe',lambda msg:ordinary.append(msg.data),1)
    publisher = node.create_publisher(String,'metadata_probe',1)
    try:
        deadline = time.monotonic()+5.
        while time.monotonic()<deadline and not (received and ordinary):
            publisher.publish(String(data='sample'))
            executor.spin_once(timeout_sec=.05)
        assert received and ordinary and received[0][0]==ordinary[0]=='sample'
        info = received[0][1]
        assert info['source_timestamp']>0
        assert 'received_timestamp' in info  # Some RMWs legitimately report zero.
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
