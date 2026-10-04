import math
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class MoveBaseNavigator:
    """Drop-in replacement for MoveToPointController.navigate_to() using move_base."""

    def __init__(self, frame="odom", refine=None, timeout=180.0):
        self.frame = frame
        self.refine = refine          # optional MoveToPointController for final alignment
        self.timeout = timeout
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[MoveBaseNav] Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("[MoveBaseNav] Connected.")

    def _current_xy(self):
        msg = rospy.wait_for_message("/odom", Odometry, timeout=3.0)
        return msg.pose.pose.position.x, msg.pose.pose.position.y

    def navigate_to(self, gx, gy, goal_yaw=None, pos_tolerance=None, label="Target"):
        # goal_yaw=None (transit): face the direction of travel
        if goal_yaw is None:
            cx, cy = self._current_xy()
            yaw = math.atan2(gy - cy, gx - cx)
        else:
            yaw = math.radians(goal_yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))

        rospy.loginfo(">>> [move_base] Navigating to %s: (%.2f, %.2f) yaw %.1f deg",
                      label, gx, gy, math.degrees(yaw))
        self.client.send_goal(goal)

        if not self.client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logwarn("[move_base] %s timed out, cancelling.", label)
            self.client.cancel_goal()
            return False

        state = self.client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("[move_base] %s failed (status %d).", label, state)
            return False

        rospy.loginfo("[move_base] %s reached (coarse).", label)

        # Fine alignment with the old controller (short distance, high precision)
        # if self.refine is not None and pos_tolerance is not None and pos_tolerance < 0.15:
        #     return self.refine.navigate_to(gx=gx, gy=gy, goal_yaw=goal_yaw,
        #                                    pos_tolerance=pos_tolerance, label=label + " (fine)")
        return True