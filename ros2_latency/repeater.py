#!/usr/bin/python3
import array
import builtin_interfaces.msg
from angsa_util.util import wrap_node
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class Repeater(Node):
    """ Subscribes to a pointcloud, copies it and re-publishes it with the current timestamp """
    def __init__(self):
        super().__init__("repeater_py")

        # You can play around with the QOS here, I could not find a significant improvement
        # reliability = ReliabilityPolicy(ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        # history = HistoryPolicy(HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)
        # liveliness = LivelinessPolicy(LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
        # durability = DurabilityPolicy(DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)
        # lifespan = Duration(seconds=0.5)
        # deadline = Duration(seconds=0.5)
        # depth = 1
        # pub_qos = QoSProfile(
        #     reliability=reliability,
        #     depth=depth,
        #     history=history,
        #     liveliness=liveliness,
        #     durability=durability,
        #     lifespan=lifespan,
        #     deadline=deadline,
        #  )

        # As the QOS doesn't really make any difference, lets keep the default one
        pub_qos = QoSProfile(depth=10)
        sub_qos = QoSProfile(depth=10)
        self._pc_sub = self.create_subscription(PointCloud2, topic="pc_source", callback=self._handle_pc, qos_profile=sub_qos)
        self._pc_pub = self.create_publisher(PointCloud2, topic="repeated_pc_py", qos_profile=pub_qos)

    def format_dt(self, t1: builtin_interfaces.msg.Time, t2: builtin_interfaces.msg.Time):
        """ Helper for formatting the difference between two stamps in microseconds """
        us1 = t1.sec * 1e6 + t1.nanosec // 1e3
        us2 = t2.sec * 1e6 + t2.nanosec // 1e3
        return f"{int(us2 - us1):5} [us]"

    def _handle_pc(self, pc: PointCloud2):
        """ Callback for the PC subscriber. """
        t1 = self.get_clock().now().to_msg()

        # Copy the pointcloud t make sure we don't have any buffer anomalies
        new_pc = pc  # Copy metadata
        new_pc.data = array.array('B', new_pc.data)  # Create new bytes array
        t2 = self.get_clock().now().to_msg()

        # Publish it
        self._pc_pub.publish(new_pc)
        t3 = self.get_clock().now().to_msg()

        # Log delta t
        self.get_logger().info(f"Age of source PC is {self.format_dt(pc.header.stamp, t1)}")
        self.get_logger().info(f"Copying took {self.format_dt(t1, t2)}")
        self.get_logger().info(f"Publishing took {self.format_dt(t2, t3)}")


def main(args=None):
    wrap_node(Repeater, args)


if __name__ == '__main__':
    main()
