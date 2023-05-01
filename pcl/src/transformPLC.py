#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from tf import TransformListener
from tf.transformations import euler_from_quaternion





class transformPCL():

    def __init__(self):
        rospy.init_node("transformPCL")

        rospy.Subscriber("/plc_centroid", Point, self.plc_callback, queue_size=10)
        
        self.centroid_world = rospy.Publisher("/plc_world_centroid", Point, queue_size=10)


        self.tf = TransformListener()
        rospy.sleep(0.5)
        print("Transform Node Initialized")            


    def plc_callback(self, msg):
        """
        Transofrmation 
        """
        x = msg.x
        y = msg.y
        z = msg.z

        # print(f"Pointcloud centroid: {msg}")

        try: 
            root_joint_name = "panda_link8" # panda_camera_optical_link
            position, quaternion = self.tf.lookupTransform("world", root_joint_name, rospy.Time())      
            r = R.from_quat(quaternion)
            w2e_T = np.hstack((r.as_matrix(), np.asarray(position).reshape((3,1))))
            w2e_T = np.vstack((w2e_T, np.array(([0,0,0,1]))))   

            e2s_T = np.array(([1,0,0,x],
                              [0,1,0,y],
                              [0,0,1,z],
                              [0,0,0,1]))
            
            w2s_T = w2e_T*e2s_T

            centroid = Point()
            centroid.x = w2s_T[0,-1]
            centroid.y = w2s_T[1,-1]
            centroid.z = w2s_T[2,-1]

            self.centroid_world.publish(centroid)


        except Exception as e:
            print(e)
            return #just retry instantly
        
if __name__ == "__main__":
    transformPCLa = transformPCL()
    rospy.spin()