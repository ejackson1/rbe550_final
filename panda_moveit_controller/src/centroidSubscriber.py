#!/usr/bin/env python
import rospy
import geometry_msgs
import numpy as np
from moveitClient import MoveitArmClient
from scipy.spatial.transform import Rotation as R


def callback(centroid):
    print("Centroid Received by Subscriber!")
    moveToCentroid(centroid.data)

def moveToCentroid(data):
        m = MoveitArmClient(init_node=True)
    
        # goalT = np.array(([1, 0, 0, -2.3],
        #                 [0, -1, 0, 0],
        #                 [0, 0, -1, 2.],
        #                 [0, 0, 0, 1]))

        # r = R.from_matrix(goalT[:3,:3])
        # quaterion = r.as_quat()

        # print(quaterion)

        pose_goal = geometry_msgs.msg.Pose()
       # pose_goal.orientation.x = quaterion[0]
       # pose_goal.orientation.y = quaterion[1]
       # pose_goal.orientation.z = quaterion[2]
       # pose_goal.orientation.w = quaterion[3]
        pose_goal.position.x = data.x
        pose_goal.position.y = data.y
        pose_goal.position.z = data.z
    #  pose_goal.position.z = 0.3
        #print("Requesting...")
        m.move_arm_EE(pose_goal)




def centroidsubscriber():
    rospy.init_node('centroidListener', anonymous=True)

    rospy.Subscriber("plc_centroid", geometry_msgs.msg.Point, callback)

    rospy.spin()

if __name__ == "__main__":
    centroidsubscriber()