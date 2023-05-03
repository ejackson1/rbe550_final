#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from panda_moveit_controller.srv import moveToPose, moveToAngles, grip, getJointAngles
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from math import pi, isclose, sin, cos, radians
import numpy as np
from scipy.spatial.transform import Rotation as R

class MoveitArmClient:
    def __init__(self, init_node=False) -> None:

        if init_node:
            rospy.init_node("moveit_client", anonymous=True)

        tau = 2.0 * pi
        kortex_home_angles = [0, -tau / 15, 0, tau / 4, 0, tau / 6, 0]   
        panda_home_angles = [0, -tau / 6, 0, -tau / 3, 0, tau / 6, 0] 

        self.arm_name = "panda"
        self.arm_base_link_name = 'panda_link0'
        self.home_joint_angles = panda_home_angles 

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)

    def move_arm_EE(self, pose):
        # client side used to move the arm to a desired EE pose

        rospy.wait_for_service('/move_it_EE')
        try:
            c = rospy.ServiceProxy('/move_it_EE', moveToPose)

            # # translate to base link reference frame
            # world2Base = self.tfBuffer.lookup_transform('world', self.arm_base_link_name, rospy.Time())
            # print(world2Base.transform.translation)

            # # ensure world to base transform is not 0,0,0
            # while world2Base.transform.translation.x == 0.0 and world2Base.transform.translation.y == 0.0:
            #     world2Base = self.tfBuffer.lookup_transform('world', self.arm_base_link_name, rospy.Time())
            #     print(world2Base.transform.translation)

            # pose.position.x += world2Base.transform.translation.x
            # pose.position.y += world2Base.transform.translation.y
            # pose.position.z += world2Base.transform.translation.z


            success = c(pose)
            bool = Bool()
            bool.data = True
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        
        except (rospy.ServiceException, tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Service called failed: %s"%e)
            return

    def move_arm_angles(self, joints):
        # client side used to move the arm to a desired set of joint angles
        rospy.wait_for_service('/move_it_angles')
        try:
            c = rospy.ServiceProxy('/move_it_angles', moveToAngles)
            success = c(joints)
            bool = Bool()
            bool.data = True
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        
        except Exception as e:
            print("Service called failed as: %s"%e)

    def move_gripper(self, finger):
        # client side used to move the grippers to a desired position
        rospy.wait_for_service('/move_it_gripper')
        try:
            c = rospy.ServiceProxy('/move_it_gripper', grip)
            success = c(finger)
            bool = Bool()
            bool.data = True
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        except Exception as e:
            print("Service called failed as: %s"%e)

    def send_arm_home(self):
        self.move_arm_angles(self.home_joint_angles)
    
    def is_arm_in_home_pos(self):
        rospy.wait_for_service('/get_joint_angles')
        try:
            c = rospy.ServiceProxy('/get_joint_angles', getJointAngles)
            response = c() # requests arm joint angles

            if response.success.data: # if service doesn't fail
                return self.are_joints_equal(response.angles, self.home_joint_angles) # compare current joint angles with home angles
            else:
                print(str(response.data.data) + ", unable to get current arm joints")
                return False
        except Exception as e:
            print("Service called failed as: %s"%e)
            return False
    
    def are_joints_equal(self, joints1, joints2, tol=.1):
        # compares each current joint angle to home joint angle - if all match within tolerance then arm is at home 
        return all([isclose(x, y, rel_tol=tol, abs_tol=tol) for x,y in zip(joints1, joints2)])

def makePose(goalT): 
    # Accepts a homogeneous transformation matrix and outputs a Pose rosmessage
    r = R.from_matrix(goalT[:3,:3])
    quaterion = r.as_quat()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaterion[0]
    pose_goal.orientation.y = quaterion[1]
    pose_goal.orientation.z = quaterion[2]
    pose_goal.orientation.w = quaterion[3]
    pose_goal.position.x = goalT[0,3]
    pose_goal.position.y = goalT[1,3]
    pose_goal.position.z = goalT[2,3]
    print(f"In makePose goalT:\n {goalT}")
    print(f"In makePose pose_goal:\n {pose_goal}")
    print("Requesting.. ")
    
    return pose_goal

def RX(goalT,angle):
    R_T = goalT[:3,:3]
    Rx = np.array(([1,0,0],
                   [0,cos(radians(angle)),-sin(radians(angle))],
                   [0,sin(radians(angle)),cos(radians(angle))]))
    goalT[:3,:3] = R_T @ Rx
    return goalT

def RY(goalT,angle):
    R_T = goalT[:3,:3]
    Ry = np.array(([cos(radians(angle)),0,sin(radians(angle))],
                   [0,1,0],
                   [-sin(radians(angle)),0,cos(radians(angle))]))
    goalT[:3,:3] = R_T @ Ry
    return goalT

def RZ(goalT,angle):
    R_T = goalT[:3,:3]
    Rz = np.array(([cos(radians(angle)),-sin(radians(angle)),0],
                   [sin(radians(angle)),cos(radians(angle)),0],
                   [0,0,1]))
    goalT[:3,:3] = R_T @ Rz
    return goalT



if __name__ == "__main__":
    m = MoveitArmClient(init_node=True)
 
    # Panda finger
    finger = 0.054
    m.move_gripper(finger)

    # m.is_arm_in_home_pos()
    # init = [1.383035467921843, -1.5321861096780172, -1.7988014563683827, -1.7618231679666039, -1.5773894447485612, 1.8261655306549525, -0.7708965437742608]
    raised = [1.3500737407279217, -1.4550843634153168, -1.69247116861691, -1.8027891555282816, -1.486084602669667, 1.7387179465620655, -0.7726231094421925]
    # final = [2.7512211763054086, -1.3382896714015704, -1.6315563705904341, -1.6254787932375034, -1.322917349269738, 1.5876109064986714, 0.36519625187956084]
    
    # move fingers around cube
    #m.move_arm_angles(init)
    goalT = np.array(([1, 0, 0, 0.5],
                      [0, -1, 0, -0.4],
                      [0, 0, -1, 0.235],
                      [0, 0, 0, 1]))
    goalT = RZ(goalT,45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)

    # pick up 
    m.move_gripper(0.01)
    #m.move_arm_angles(raised)
    

    # Travel Through door
    #m.move_arm_angles(final)
    goalT = np.array(([1, 0, 0, 0.5],
                      [0, -1, 0, 0.4],
                      [0, 0, -1, 0.27],
                      [0, 0, 0, 1]))
    goalT = RZ(goalT,45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
  
    # Release
    m.move_gripper(finger)