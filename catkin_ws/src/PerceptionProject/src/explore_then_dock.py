import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

class Navigator:

    def __init__(self, poses):

        rospy.init_node('send_goal_pose', disable_signals = True)

        self.poses = poses
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        self.latest_aruco_pos = None


    def found_goal_callback(self, data):
        self.latest_aruco_pos = [data.pose.position.x,data.pose.position.y]


    def send_goal_pose(self, pos):
        # https://www.programcreek.com/python/example/113987/move_base_msgs.msg.MoveBaseGoal
        # this page taks about how to set the goal pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pos[0]
        goal.target_pose.pose.position.y = pos[1]


        # need to normalize quarternion
        # forum https://answers.ros.org/question/28819/quaternion-has-length-close-to-zero-discarding-as-navigation-goal-why/
        # solution https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/
        quat = Quaternion(*quaternion_from_euler(0, 0, 1.57))
        goal.target_pose.pose.orientation = quat
        # quaternionArray = tf.transformations.quaternion_about_axis(0, (0,0,1))
        # goal.target_pose.pose.orientation = self.array_to_quaternion(quaternionArray)
        self.move_client.send_goal(goal)


        wait = self.move_client.wait_for_result()
        if not wait:
            rospy.signal_shutdown("Error connecting to move base server")
        else:
            return self.move_client.get_result()


    def navigate(self):
        
        arcuo_detector = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.found_goal_callback)

        for p in self.poses:
            self.send_goal_pose(p)
        

        rospy.loginfo(f'Aruco Pos is {self.latest_aruco_pos}')
        self.send_goal_pose(self.latest_aruco_pos)


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    poses = [
        [-1, -2],
        [1, -2],
        [2, -1],
        [2, 1],
        [1, 2],
        [-1, 2],
        [-2, 1]
    ]

    # poses = [
    #     [0.5, 1.5],
    #     [1.5, 1.5],
    #     [2.5, 0.5],
    #     [1.5, 1.5],
    #     [0.5, 1.5]
    # ]

    navigator = Navigator(poses)
    navigator.navigate()


