import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
import time
import pybullet as p

class BotPluginMoveIt:
    def __init__(self, environment, move_group = 'Manipulator'):
        self.env = environment
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("bot_node_moveit", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(move_group)
        # display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.pose_target = geometry_msgs.msg.Pose()



    def plan_motion(self, x, y, z, roll, pitch, yaw):
        self.pose_target.position.x = x
        self.pose_target.position.y = y
        self.pose_target.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # self.pose_target.orientation.x = quaternion[0]
        # self.pose_target.orientation.y = quaternion[1]
        # self.pose_target.orientation.z = quaternion[2]
        # self.pose_target.orientation.w = quaternion[3]
        self.pose_target.orientation.w = quaternion[3]
        
        
        self.group.set_pose_target(self.pose_target)
        self.plan = self.group.plan()


    def set_starting_state(self, state):
        self.group.set_start_state(state)


    def execute_plan(self):
        for step in self.plan[1].joint_trajectory.points:
            joint_angles = step.positions
            for i, angle in enumerate(joint_angles):
                p.setJointMotorControl2(bodyIndex=self.env.panda_id,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=angle,
                                       )
                p.stepSimulation()
                # link_state = p.getLinkState(self.env._robotic_arm, 6, computeLinkVelocity = True)
                #linear_velocity = link_state[6]
                #angular_velocity = link_state[7]
                #current_time = time.time()
                #with open('vel_data.csv', 'a') as f:
                #    f.write(f'{linear_velocity[0]},{linear_velocity[1]},{linear_velocity[2]},{angular_velocity[0]},{angular_velocity[1]},{angular_velocity[2]},{current_time}\n')

                time.sleep(1/240.0)
            
        # self.group.go(wait = True)
