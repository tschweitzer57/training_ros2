# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from uav_interfaces.msg import Groundtruth
from tutorial_interfaces.msg import Num


class MSubscriber(Node):

    def __init__(self):
        super().__init__('m_subscriber')
        self.subscription_num = self.create_subscription(Num,'topic1',self.listener_callback,10)
        self.subscription_num  # prevent unused variable warning
        self.subscription_gt = self.create_subscription(Groundtruth,'topic2',self.listener2_callback,10)
        self.subscription_gt # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.num)

    def listener2_callback(self, msg):
        self.get_logger().info('I heard: frame: %d \tx: %d \ty: %d \tz: %d \tpsi: %d' % (msg.frame_id, msg.x, msg.y, msg.z, msg.psi))


def main(args=None):
    rclpy.init(args=args)

    mult_subscriber = MSubscriber()

    rclpy.spin(mult_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mult_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
