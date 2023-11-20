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

from tutorial_interfaces.msg import Num
from uav_interfaces.msg import Groundtruth


class MultiPublisher(Node):

    def __init__(self):
        super().__init__('multi_publisher')
        self.publisher_Num = self.create_publisher(Num, 'topic1', 10)
        self.publisher_gt = self.create_publisher(Groundtruth, 'topic2', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg1 = Num()
        msg1.num = self.i
        self.publisher_Num.publish(msg1)
        self.get_logger().info('Publishing: "%s"' % msg1.num)

        msg2 = Groundtruth()
        msg2.frame_id = self.i
        msg2.x = 0.452
        msg2.y = 0.587
        msg2.z = 0.897
        msg2.psi = 58.67
        self.publisher_gt.publish(msg2)
        self.get_logger().info('Publishing: frame: %d \tx: %f \ty: %f \tz: %f \tpsi: %f' % (msg2.frame_id, msg2.x, msg2.y, msg2.z, msg2.psi))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    multi_publisher = MultiPublisher()

    rclpy.spin(multi_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multi_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
