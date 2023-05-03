#!/usr/bin/env python3

import rospy
import open3d
from open3d import geometry, utility
import numpy as np
from ctypes import * # convert float to uint32
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf import TransformListener 


class PLC_stitch():

    def __init__(self) -> None:
        rospy.init_node("PLC_segement")

        rospy.Subscriber("/plc_outliers", PointCloud2, self.plc_cb, queue_size=10)
        self.plc_pub = rospy.Publisher("/plc_outliers2", PointCloud2, queue_size=10)

        self.plc_PC = PointCloud2()
        self.plc_PC.header.frame_id = "world"
        self.plc_PC.point_step = 16
        self.plc_PC.height = 1

        self.open3d_cloudINIT = geometry.PointCloud()
        self.tf = TransformListener()
        # pcl.
        rospy.set_param("/add_PCL", False)
        rospy.sleep(0.5)
        print("Initlaized PCL Stitching Node")

        
        pass

    def plc_cb(self, msg):
        """
        PLC_CB gives us a PointCloud2 Object that has already been segemented. We can add data points to this naively 
        """
        if rospy.get_param("/add_PCL") == True:
            # self.plc_PC.width += msg.width
            # OGSize = len(self.plc_PC.data)
            # self.plc_PC.data
            # # self.plc_PC.height += msg.height
            # self.plc_PC.row_step += msg.row_step
            # self.plc_PC.fields = msg.fields
            # self.plc_PC.data += msg.data
            
            print(f"Origional PointCloud: {msg.header.frame_id}")
            open3d_cloud = self.convertCloudFromRosToOpen3d(msg)
            # print(f"Cloud: {open3d_cloud}")
            # print(f"Cloud type: {type(open3d_cloud)}")
            self.open3d_cloudINIT += open3d_cloud

            # Try calculating centroid of current model
            center = self.open3d_cloudINIT.get_center()
            print(f"Center: {center}")
            
            ros_cloud = self.convertCloudFromOpen3dToRos(self.open3d_cloudINIT, frame_id=msg.header.frame_id)
            # print(f"ROS_cloud: {ros_cloud}")
            
            # self.plc_PC += msg
            rospy.set_param("/add_PCL", False)

        # print(f"Data: {msg.height}")
            self.plc_pub.publish(ros_cloud)
        
    def convertCloudFromOpen3dToRos(self, open3d_cloud, frame_id="world"):
        # The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        FIELDS_XYZ = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        FIELDS_XYZRGB = FIELDS_XYZ + \
            [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
                
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        print(f"inside type: {type(open3d_cloud)}")
        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.points)
        if not open3d_cloud.colors: # XYZ only
            fields=FIELDS_XYZ
            cloud_data=points
        else: # XYZ + RGB
            fields=FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.
            colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
            colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
            cloud_data=np.c_[points, colors]
        
        # create ros_cloud
        return pc2.create_cloud(header, fields, cloud_data)
    
    def convertCloudFromRosToOpen3d(self, ros_cloud):
        # Bit operations
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        convert_rgbUint32_to_tuple = lambda rgb_uint32: (
            (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
        )
        convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
            int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

        # Check empty
        open3d_cloud = geometry.PointCloud()
        
        # open3d.geometry
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            open3d_cloud.points = utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

if __name__ == "__main__":
    open3d_a = PLC_stitch()
    rospy.spin()