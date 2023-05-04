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
from tf import TransformListener
from geometry_msgs.msg import Vector3

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
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.tf = TransformListener()
        rospy.sleep(1)

    def move_arm_EE(self, pose):
        # client side used to move the arm to a desired EE pose

        position, quaternion = self.tf.lookupTransform("panda_link8", "panda_hand", rospy.Time())      
        r = R.from_quat(quaternion)
        print(f"r.as_matrix() {r.as_matrix()}")
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
    # usage: goalT = RX(goalT,45)
    R_T = goalT[:3,:3]
    Rx = np.array(([1,0,0],
                   [0,cos(radians(angle)),-sin(radians(angle))],
                   [0,sin(radians(angle)),cos(radians(angle))]))
    goalT[:3,:3] = Rx @ R_T
    return goalT

def RY(goalT,angle):
    # usage: goalT = RY(goalT,45)
    R_T = goalT[:3,:3]
    Ry = np.array(([cos(radians(angle)),0,sin(radians(angle))],
                   [0,1,0],
                   [-sin(radians(angle)),0,cos(radians(angle))]))
    goalT[:3,:3] = Ry @ R_T
    return goalT

def RZ(goalT,angle):
    # usage: goalT = RZ(goalT,45)
    R_T = goalT[:3,:3]
    Rz = np.array(([cos(radians(angle)),-sin(radians(angle)),0],
                   [sin(radians(angle)),cos(radians(angle)),0],
                   [0,0,1]))
    goalT[:3,:3] =  Rz @ R_T
    return goalT

def envScan(goalT):
    goalT = RX(goalT,-10)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    # Point cloud stitch current view
    rospy.set_param("/add_PCL", True)
    rospy.sleep(.5)
    centroid = CENTROID
    print(f"CENTROID: {centroid}")


    # Look 45 deg +X 
    goalT = RX(goalT,55)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    # Point cloud stitch current view
    rospy.set_param("/add_PCL", True)

    # Look 45 deg +X 
    goalT = RX(goalT,45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    # Point cloud stitch current view
    rospy.set_param("/add_PCL", True)

    # Look 45 deg -Z 
    goalT = RZ(goalT,-45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    # Point cloud stitch current view
    rospy.set_param("/add_PCL", True)

    # Look 45 deg -X 
    goalT = RX(goalT,-45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    # Point cloud stitch current view
    rospy.set_param("/add_PCL", True)

    # Look 45 deg -X 
    goalT = RX(goalT,-45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    # Point cloud stitch current view
    rospy.set_param("/add_PCL", True)

    return centroid

def getCentroid(centroid):
    global CENTROID
    CENTROID = centroid

if __name__ == "__main__":
    m = MoveitArmClient(init_node=True)
    rospy.Subscriber("/plc_centroid2", Vector3, getCentroid, queue_size=1)
 
    # Panda open fingers
    finger = 0.055
    m.move_gripper(finger)

    # Initial Position for scan
    goalT = np.array(([1, 0, 0, 0.495],
                      [0, -1, 0, -0.4],
                      [0, 0, -1, 0.5],
                      [0, 0, 0, 1]))

    centroid = envScan(goalT)
    print(f"centroid: {centroid}")

    # move above cube
    goalT = np.array(([1, 0, 0, 0.495],
                      [0, -1, 0, -0.4],
                      [0, 0, -1, 0.5],
                      [0, 0, 0, 1]))
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    m.move_gripper(finger)

    # move fingers around cube
    goalT = np.array(([1, 0, 0, centroid.x],
                      [0, -1, 0, centroid.y],
                      [0, 0, -1, centroid.z+.09],
                      [0, 0, 0, 1]))
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    m.move_gripper(finger)

    # goalT = np.array(([1, 0, 0, centroid.x-0.088],
    #                     [0, -1, 0, centroid.y+0.1],
    #                     [0, 0, -1, centroid.z + 0.081],
    #                     [0, 0, 0, 1]))
    
    # pick up 
    m.move_gripper(0.006)
    goalT = np.array(([1, 0, 0, centroid.x],
                      [0, -1, 0, centroid.y],
                      [0, 0, -1, 0.5],
                      [0, 0, 0, 1]))
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
    m.move_gripper(0.006)

    # Travel Through door
    #m.move_arm_angles(final)
    goalT = np.array(([1, 0, 0, 0.5],
                      [0, -1, 0, 0.4],
                      [0, 0, -1, 0.4],
                      [0, 0, 0, 1]))
    goalT = RZ(goalT,45)
    pose_goal = makePose(goalT)
    m.move_arm_EE(pose_goal)
  
    # Release
    m.move_gripper(finger)

    # Reset Arm
    # goalT = np.array(([1, 0, 0, 0.495],
    #                   [0, -1, 0, -0.4],
    #                   [0, 0, -1, 0.5],
    #                   [0, 0, 0, 1]))
    # pose_goal = makePose(goalT)
    # m.move_arm_EE(pose_goal)