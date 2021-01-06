
from flow_net.flow_net_ros import FlowNetRos

import rospy


if __name__ == "__main__":
    rospy.init_node("kitti_comparison_flow")

    flow_net = FlowNetRos()