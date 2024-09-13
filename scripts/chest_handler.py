#!/usr/bin/env python
"""

Author(s):
    1. Lorena Genua (lorena.genua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.

"""

# # Standart libraries:
import rospy
import numpy as np

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Int32,
    Float32,
)
from geometry_msgs.msg import (
    Pose,
)
from std_srvs.srv import (
    Empty,
    SetBool,
)
from gopher_ros_clearcore.srv import (
    MovePosition,
)
from Scripts.srv import (UpdateState, UpdateChest)


class ChestHandler:
    """
    
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name

        # # Public CONSTANTS:

        # # Private variables:
        self.__state = 0
        self.__chest_position = 0.44
        self.__desired_position = 0.44
        self.__adjusting = False

        self.__chest_camera = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__tf_from_anchor_to_object = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
        }

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            'chest_control': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['chest_control'] = (
            rospy.Subscriber(
                f'/chest_control/is_initialized',
                Bool,
                self.__chest_control_callback,
            )
        )
        rospy.Subscriber(
            '/my_gen3/target_hologram',
            Pose,
            self.__tf_anchor_object_callback,
        )
        rospy.Subscriber(
            'chest_logger/current_position',
            Float32,
            self.__chest_position_callback,
        )
        rospy.Subscriber(
            f'/my_gen3/robot_control/current_task_state',
            Int32,
            self.__robot_pick_and_place_callback,
        )
        rospy.Subscriber(
            'my_gen3/tf_chest_cam_anchor',
            Pose,
            self.__chest_cam_transform_callback,
        )
        # # Service provider:
        rospy.Service(
            f'{self.__NODE_NAME}/adjust_chest',
            Empty,
            self.__adjust_chest,
        )
        rospy.Service(
            '/move_chest',
            UpdateState,
            self.__move_chest_interface,
        )

        # # Service subscriber:
        self.__chest_home = rospy.ServiceProxy(
            '/chest_control/home',
            Empty,
        )
        self.__chest_stop = rospy.ServiceProxy(
            '/chest_control/stop',
            Empty,
        )
        self.__move_chest = rospy.ServiceProxy(
            '/chest_control/move_absolute_position',
            MovePosition,
        )

        # rospy.wait_for_service('object_tracker/pause')

        self.__pause_object_tracking = rospy.ServiceProxy(
            '/object_tracker/pause',
            SetBool,
        )
        # # Topic publisher:

        # # Topic subscriber:
        # # Timers:
        # rospy.Timer(
        #     rospy.Duration(1.0 / 100),
        #     self.__some_function_timer,
        # )

        rospy.sleep(3)

        self.__home_and_pause_tracking()

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __chest_control_callback(self, message):
        """Monitors /chest_control/is_initialized topic.
        
        """

        self.__dependency_status['chest_control'] = message.data

    def __robot_pick_and_place_callback(self, message):

        self.__state = message.data

    def __tf_anchor_object_callback(self, message):

        self.__tf_from_anchor_to_object['position'][0] = message.position.x
        self.__tf_from_anchor_to_object['position'][1] = message.position.y
        self.__tf_from_anchor_to_object['position'][2] = message.position.z

        self.__tf_from_anchor_to_object['orientation'][0
                                                      ] = message.orientation.w
        self.__tf_from_anchor_to_object['orientation'][1
                                                      ] = message.orientation.x
        self.__tf_from_anchor_to_object['orientation'][2
                                                      ] = message.orientation.y
        self.__tf_from_anchor_to_object['orientation'][3
                                                      ] = message.orientation.z

    def __chest_cam_transform_callback(self, message):

        self.__chest_camera['position'][0] = message.position.x
        self.__chest_camera['position'][1] = message.position.y
        self.__chest_camera['position'][2] = message.position.z

    def __chest_position_callback(self, message):
        self.__chest_position = message.data

        if self.__adjusting and abs(
            self.__chest_position - self.__desired_position
        ) < 0.05:
            self.__pause_object_tracking(False)
            self.__adjusting = False

    # # Service handlers:
    def __adjust_chest(self, request):
        """

        """

        # TO DO: when using anchor, change from [2] to [1]
        object_height = self.__tf_from_anchor_to_object['position'][1]
        camera_height = self.__chest_camera['position'][1]
        difference_height = camera_height - object_height

        if self.__state == 3:

            if self.__is_chest_highest_position():
                self.__desired_position = 0.20
            else:
                return []

        else:

            if difference_height > 0.14 and self.__is_chest_middle_position():
                self.__desired_position = 0.44

            elif difference_height < -0.14 and self.__is_chest_highest_position(
            ):
                self.__desired_position = 0.20

            else:
                return []

        self.__move_and_pause_tracking(self.__desired_position)

        return []

    def __move_chest_interface(self, request):

        if request.state == 0 and self.__is_chest_middle_position():
            self.__desired_position = 0.44

        elif request.state == 1 and self.__is_chest_highest_position():
            self.__desired_position = 0.20

        else:
            return True

        self.__move_and_pause_tracking(self.__desired_position)

        return True

    def __is_chest_highest_position(self):

        if abs(self.__chest_position - 0.44) < 0.03:
            return True
        else:
            return False

    def __is_chest_middle_position(self):

        if abs(self.__chest_position - 0.20) < 0.03:
            return True
        else:
            return False

    # # Topic callbacks:

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __move_and_pause_tracking(self, position):

        self.__adjusting = True
        self.__pause_object_tracking(True)

        self.__move_chest(position, 1.0)

    def __home_and_pause_tracking(self):
        self.__adjusting = True
        self.__pause_object_tracking(True)

        self.__chest_home()

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.
        self.__chest_stop()

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'chest_handler',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    class_instance = ChestHandler(node_name=node_name,)

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
