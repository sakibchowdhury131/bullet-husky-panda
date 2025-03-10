import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
import time
import pybullet as p
from sensor_msgs.msg import JointState


class BotPluginMoveIt:
    def __init__(self, environment, move_group = 'arm', planner_id = 'RRTConnectConfigDefault', max_planning_time = 1.0):
        self.env = environment
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("bot_node_moveit", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(move_group)
        # display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.group.set_planner_id(planner_id)
        self.group.set_planning_time(max_planning_time)
        self.pose_target = geometry_msgs.msg.Pose()
        self.success = False
        self.plan = None
        



    def plan_motion(self, x, y, z, roll = 0, pitch = 0, yaw = np.pi/2, ignore_pose_control = False):
        self.pose_target.position.x = x
        self.pose_target.position.y = y
        self.pose_target.position.z = z

        if ignore_pose_control:
            self.pose_target.orientation.w = 1
        else:
            
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            self.pose_target.orientation.x = quaternion[0]
            self.pose_target.orientation.y = quaternion[1]
            self.pose_target.orientation.z = quaternion[2]
            
            self.pose_target.orientation.w = quaternion[3]
        # 
        
        
        self.group.set_pose_target(self.pose_target)
        self.success, self.plan, _, _ = self.group.plan()


    def set_starting_state(self, state):
        self.group.set_start_state(state)
    


    def execute_plan(self):
        for step in self.plan.joint_trajectory.points:
            joint_angles = step.positions
            for i, angle in enumerate(joint_angles):
                p.setJointMotorControl2(bodyIndex=self.env.panda_id,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=angle,
                                       )
                p.stepSimulation()
                time.sleep(self.env.time_step)
            
        # self.group.go(wait = True)
