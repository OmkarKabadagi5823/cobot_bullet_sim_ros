import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, ExecuteTrajectoryActionFeedback
from actionlib_msgs.msg import GoalID, GoalStatus
from sensor_msgs.msg import JointState

class URExecutionManager():
    def __init__(self):
        self.__init_params()
        self.__init_publishers()
        self.__init_subscribers()

    def __init_params(self):
        self._goal_id = GoalID()
        self._execution_status = GoalStatus()
        self._joint_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __init_publishers(self):
        self._cancel_pub = rospy.Publisher(
            '/execute_trajectory/cancel',
            GoalID,
            queue_size=10
        )

    def __init_subscribers(self):
        self._goal_sub = rospy.Subscriber(
            '/execute_trajectory/goal', 
            ExecuteTrajectoryActionGoal, 
            self.__action_goal_callback
        )

        self._staus_sub = rospy.Subscriber(
            '/execute_trajectory/feedback',
            ExecuteTrajectoryActionFeedback,
            self.__action_feedback_callback
        )

        self._joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.__joint_state_callback,
        )

    def __action_goal_callback(self, msg):
        self._goal_id = msg.goal_id
        

    def __action_feedback_callback(self, msg):
        self._execution_status = msg.status

    def __joint_state_callback(self, msg):
        self._joint_state = msg.position

    def get_joint_state(self):
        return self._joint_state

    def cancel_execution(self):
        if self._execution_status.status == GoalStatus.ACTIVE:
            self._cancel_pub.publish(self._execution_status.goal_id)
            rospy.loginfo_throttle(1, "Cancelling trajectory execution")
            return True
        else:
            rospy.logwarn_throttle(5, "UR Robot not active, so no execution to cancel ( Status: {} )".format(self._execution_status.text))
            return False