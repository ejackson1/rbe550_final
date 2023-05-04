#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from panda_moveit_controller.srv import moveToPose, moveToPoseResponse, moveToAngles, moveToAnglesResponse, grip, gripResponse, getJointAngles, getJointAnglesResponse
from panda_moveit_controller.srv import sendPIUpdate
from control_msgs.msg import GripperCommandActionGoal
from std_msgs.msg import Bool

class MoveItPlanner:
    def __init__(self) -> None:     
        # Initialize nodes and service
        rospy.init_node("move_group_interface", anonymous=True)

        # MoveIt Services with custom messages
        sEE = rospy.Service('/move_it_EE', moveToPose, self.move_arm)
        sAngs = rospy.Service('/move_it_angles', moveToAngles, self.move_arm_angle)
        sGrip = rospy.Service('/move_it_gripper', grip, self.moveGripper)
        sPose = rospy.Service('/get_joint_angles', getJointAngles, self.get_arm_pose)

        sPlanningUpdate = rospy.Service('/update_PI', sendPIUpdate, self.updatePI)
        
        # Robot arm groups and information
        self.arm_name = "panda"
        self.dof = 7
        print("Initalize MoveIt for " + self.arm_name + " in namespace " + rospy.get_namespace())

        # launch arm groups with arm specific arguments 
        # Initialize roscpp
        joint_state_topic = ['joint_states:=/panda/joint_states'] # specific to panda
        moveit_commander.roscpp_initialize(joint_state_topic)

        # Initalize moveit commander and necessary moveit utilities
        self.robot = moveit_commander.RobotCommander()
        self.arm_group_name = "panda_arm"
        

        # Gripper related subscriptions and publications
        self.js_sub = rospy.Subscriber("/panda/joint_states", JointState, self.js_cb)
        self.left_finger_pub = rospy.Publisher("/panda/panda_finger1_controller/command", Float64, queue_size=1000)
        self.right_finger_pub = rospy.Publisher("/panda/panda_finger2_controller/command", Float64, queue_size=1000)


        # Common moveit setup
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.scene = moveit_commander.PlanningSceneInterface()
        
        
        
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

        
        self.TOLERANCE = 0.005 # 1 cm of accuracy

        # helpful debugging commands 
        print(rospy.get_namespace())

        planning_frame = self.arm_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.arm_group.set_end_effector_link("panda_hand")
        eef_link = self.arm_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        rospy.loginfo("Ready to accept poses!")

    def updatePI(self, co):

        self.scene.add_object(co)



    def js_cb(self, js):
         # Global callback for joint states
        global joint_states
        joint_states = js
    

    def get_cartesian_pose(self, arm_group):
        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def move_arm(self, pose):
        # Move the arm to a specfic EE pose 
        # pose contains both a linear [xyz] and angular position [xyzw]
        bool = Bool()
        try:
            # Current Pose before arm movement
            previous_pose = self.arm_group.get_current_pose()
            previous_pose = previous_pose.pose
            self.get_cartesian_pose(self.arm_group)
            
            # Plan and go to desired EE pose
            self.arm_group.set_goal_position_tolerance(self.TOLERANCE)
            self.arm_group.set_pose_target(pose.pose)
            rospy.loginfo("Planning and going to waypoint")
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.get_cartesian_pose(self.arm_group)

            # Return success to the service
            bool.data = True
            print("Success")
            return moveToPoseResponse(bool)

        except Exception as e: #something error'd, return a failure to the service
            print(e)

            bool.data = False
            return moveToPoseResponse(bool)

    def move_arm_angle(self, msg):
        # Move the arms to a desired set of angles 
        bool = Bool()
        
        if len(msg.angles) != self.dof: # check if the message contains the correct number of angles to send to each joint
           rospy.loginfo("Error! Not the corrent amount of dofs! Should be {}".format(self.dof))
           raise Exception("Error! Not the corrent amount of dofs! Should be {}".format(self.dof))

        try:
            # Report what the current joint angles are
            joint_current = self.arm_group.get_current_joint_values()
            joints = [x for x in msg.angles]
            rospy.loginfo("Current joint values are: {}. \n Attempting to correct them!".format(joint_current))
            rospy.loginfo("\n Sending joint angles: {}".format(msg.angles))

            # Send to desired joint angles, stop when finished, report cartesian pose
            self.arm_group.go(joints, wait=True) 
            self.arm_group.stop()
            self.get_cartesian_pose(self.arm_group)

            # Return success to service
            bool.data = True
            print("Success!")
            return moveToAnglesResponse(bool)
        
        except Exception as e: # something error'd send failure to service
            print(e)
            bool.data = False
            return moveToAnglesResponse(bool)

    def moveGripper(self, msg):
        # Service to move the gripper to a desired position from the center
        bool = Bool()
        try:
            if self.arm_name == "panda": # panda specific protocal for gripping
                
                # Declare left and right fingers, report current position 
                left = self.robot.get_joint('panda_finger_joint1')
                right = self.robot.get_joint('panda_finger_joint2')
                cur_left = joint_states.position[joint_states.name.index("panda_finger_joint1")]
                cur_right = joint_states.position[joint_states.name.index("panda_finger_joint2")]
                print(f"Left current: {cur_left}\tRight current: {cur_right}")

                # Check if the sent gripper position is within bounds
                if msg.position <= left.max_bound() and msg.position >= left.min_bound() and msg.position <= right.max_bound() and msg.position >= right.min_bound():
                    print(f"Sending gripper to position: {msg.position}!")
                    self.left_finger_pub.publish(msg.position)
                    self.right_finger_pub.publish(msg.position)
                    bool.data = True
                    return gripResponse(bool)
                
                else: # not within bounds
                    rospy.loginfo(f"Gripping conditions not within scope. Try a value between 0 and {left.max_bound()}!")
                    bool.data = False
                    return gripResponse(bool)
            
            else: # kortex
                
                # Declare finger joint and report current position
                finger_joint = self.robot.get_joint('finger_joint')
                curr_finger_joint = joint_states.position[joint_states.name.index("finger_joint")]
                print(f"Current Finger Position: {curr_finger_joint}")

                # Check if the sent gripper position is within bounds
                if msg.position >= finger_joint.min_bound() and msg.position <= finger_joint.max_bound(): # check if in range
                    print(f"Sending gripper to position: {msg.position}!")
                    gripperAction = GripperCommandActionGoal()
                    gripperAction.goal.command.position = msg.position
                    self.gen3_gripper_pub.publish(gripperAction)
                    bool.data = True
                    return gripResponse(bool)
                
                else: # not within bounds
                    rospy.loginfo(f"Gripping conditions not within scope. Try a value between 0 and {finger_joint.max_bound()}!")
                    bool.data = False
                    return gripResponse(bool)
        
        except Exception as e:
            rospy.loginfo(e)
            bool.data = False
            return gripResponse(bool)

    def get_arm_pose(self, msg):
        bool = Bool()
        joints = [None]*7

        try:
            joints = self.arm_group.get_current_joint_values()
            bool.data = True
            return getJointAnglesResponse(joints, bool)

        except Exception as e:
            rospy.loginfo(e)
            bool.data = False
            return getJointAnglesResponse(joints, bool)


if __name__ == "__main__":
    moveitplanner = MoveItPlanner()
    rospy.spin()
