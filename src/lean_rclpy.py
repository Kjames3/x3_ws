"""ROS node without the default incompatible-QoS event waitables."""

from rclpy.node import Node
from rclpy.qos_event import PublisherEventCallbacks, SubscriptionEventCallbacks


class LeanNode(Node):
    def create_publisher(self, msg_type, topic, qos_profile, *, event_callbacks=None, **kwargs):
        if event_callbacks is None:
            event_callbacks = PublisherEventCallbacks(use_default_callbacks=False)
        return super().create_publisher(
            msg_type, topic, qos_profile, event_callbacks=event_callbacks, **kwargs)

    def create_subscription(
            self, msg_type, topic, callback, qos_profile, *, event_callbacks=None, **kwargs):
        if event_callbacks is None:
            event_callbacks = SubscriptionEventCallbacks(use_default_callbacks=False)
        return super().create_subscription(
            msg_type, topic, callback, qos_profile, event_callbacks=event_callbacks, **kwargs)
