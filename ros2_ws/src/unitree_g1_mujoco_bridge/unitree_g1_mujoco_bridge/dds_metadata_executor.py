"""Humble executor adapter retaining metadata discarded by its Python executor.

Only designated callbacks receive the (message, info) pair. This uses Humble's
take/execute subscription hooks; DDS itself supplies both timestamps.
"""
from rclpy.executors import SingleThreadedExecutor,await_or_execute


class DDSMetadataExecutor(SingleThreadedExecutor):
    def __init__(self,metadata_callbacks=(),**kwargs):
        super().__init__(**kwargs)
        self.metadata_callbacks = tuple(metadata_callbacks)

    def _take_subscription(self,sub):
        with sub.handle:
            return sub.handle.take_message(sub.msg_type,sub.raw)

    async def _execute_subscription(self,sub,packet):
        if packet is not None:
            message,info = packet
            if sub.callback in self.metadata_callbacks:
                await await_or_execute(sub.callback,message,info)
            else:
                await await_or_execute(sub.callback,message)
