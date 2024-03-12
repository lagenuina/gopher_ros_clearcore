#!/usr/bin/env python
"""

"""

import rospy
import numpy as np

from std_msgs.msg import (Float32)
from std_srvs.srv import (
    Empty,
)
from gopher_ros_clearcore.msg import (Position)
from gopher_ros_clearcore.srv import (MovePosition)
from Scripts.srv import BoolUpdate


class HololensChestMapping:
    """
    
    """

    def __init__(self,):
        """
        
        """
        # # Public variables:
        self.chest_position = 0.44
        self.__shut_down = False

        # # ROS node:
        rospy.init_node('hololens_chest_mapping')
        rospy.on_shutdown(self.__node_shutdown)

        # # Service provider:
        self.stop_hololens_chest_node = rospy.Service(
            '/my_gen3/hololens_chest/shut_down',
            BoolUpdate,
            self.__shutdown_node_service,
        )

        # # Service subscriber:
        # self.__chest_home = rospy.ServiceProxy(
        #     'chest_control/home',
        #     Empty,
        # )
        self.__chest_stop = rospy.ServiceProxy(
            'chest_control/stop',
            Empty,
        )
        self.__chest_position = rospy.ServiceProxy(
            'chest_control/move_absolute_position',
            MovePosition,
        )

        # # Topic publisher:
        rospy.Subscriber(
            '/slider_value',
            Float32,
            self.__slider_value_callback,
        )

        # rospy.sleep(3)

        # self.__chest_home()

    # # Service handlers:
    def __shutdown_node_service(self, request):

        self.__shut_down = True

        return True

    # # Private methods:
    def __node_shutdown(self):
        """
        
        """

        print('\nNode is shutting down...\n')

        self.__chest_stop()

        print('\nNode is shut down.\n')

    def __slider_value_callback(self, message):

        self.chest_position = np.round(message.data, 2) * 0.44

        self.__chest_position(self.chest_position, 1.0)

    # # Public methods:
    def main_loop(self):
        """
        
        """
        if self.__shut_down:

            rospy.signal_shutdown("Service was called.")

            rospy.loginfo_once("\n E-Stop! Hololens chest node has shutdown.\n")

            self.__shut_down = False


def main():
    """
    
    """

    chest_mapping = HololensChestMapping()

    print('\nHololens-chest mapping is ready.\n')

    node_rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        chest_mapping.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
