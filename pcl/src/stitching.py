#!/usr/bin/env python3

import rospy
import open3d
from open3d import geometry, utility, visualization
import numpy as np
from ctypes import * # convert float to uint32

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf import TransformListener 
import copy
import matplotlib.pyplot as plt

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point


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

        rospy.set_param("/add_PCL", False)
        rospy.sleep(0.5)
        print("Initlaized PCL Stitching Node")

        
        pass
    
    def add_pointcloud_collision_object(self, pointcloud, frame_id):
        
        mesh = Mesh()
        vertices = []
        triangles = []

        for p in pc2.read_points(pointcloud):
            vertices.append(Point(x=p[0], y=p[1], z=p[2]))

        for i in range(0, len(vertices), 3):
            triangle = MeshTriangle(vertex_indices=[i, i+1, i+2])
            triangles.append(triangle)

        mesh.vertices = vertices
        mesh.triangles = triangles
        
        
        
        # # Convert pointcloud to mesh
        # mesh = Mesh()
        # for p in pc2.read_points(pointcloud, skip_nans=True, field_names=("x", "y", "z")):
        #     vertex = [p[0], p[1], p[2]]
        #     mesh.vertices.append(vertex)

        # # Create solid primitive for the mesh
        # solid_primitive = SolidPrimitive()
        # solid_primitive.type = SolidPrimitive.TRIANGLE_LIST
        # solid_primitive.dimensions.append(0) # dummy value since TRIANGLE_LIST doesn't use this field
        # solid_primitive.meshes.append(mesh)

        # Create collision object
        collision_object = CollisionObject()
        collision_object.id = "pointcloud_object"
        collision_object.header.frame_id = frame_id
        collision_object.meshes.append(mesh)
        collision_object.mesh_poses.append(Pose())
        collision_object.operation = CollisionObject.ADD

        return collision_object

    def sendCO_2_PI(self, co):
        rospy.wait_for_service('/update_PI')
        c = rospy.ServiceProxy('/update_PI', CollisionObject)

        c(co)





    def plc_cb(self, msg):
        """
        PLC_CB gives us a PointCloud2 Object that has already been segemented. We can add data points to this naively 
        """
        if rospy.get_param("/add_PCL") == True:
            open3d_cloud = self.convertCloudFromRosToOpen3d(msg)
            # print(f"Cloud: {open3d_cloud}")
            # print(f"Cloud type: {type(open3d_cloud)}")
            self.open3d_cloudINIT += open3d_cloud

            # Try calculating centroid of current model
            self.center = self.open3d_cloudINIT.get_center()
            print(f"Center: {self.center}")

            # Copy current model and find normals
            copied_o3dINIT = copy.deepcopy(self.open3d_cloudINIT)
            # # copied_o3dINIT.estimate_normals()
            voxelCloud = copied_o3dINIT.voxel_down_sample(voxel_size=0.01)
            # visualization.draw_geometries([voxelCloud], point_show_normal=True)
            # print(f"Normals?: {np.asarray(voxelCloud.points)}")
            
            print(f"tpye points: {type(copied_o3dINIT.points)}")
            ## Try to seperate point clouds with DBSClustering
            intvector = copied_o3dINIT.cluster_dbscan(eps=0.02, min_points=10, print_progress=True)
            
            # print(f"IntVector: {intvector}")
            
            pointList = utility.Vector3dVector()
            for point, intvector in zip(copied_o3dINIT.points, intvector):
                if intvector == 0:
                    pointList.append(point)
                
            self.newPCL = geometry.PointCloud(points=pointList)

            

            
            # print(f"copied_o3dINIT.points {len(copied_o3dINIT.points)}")
            # max_label = np.array(intvector).max()
            # print(f"point cloud has {max_label + 1} clusters")
            # colors = plt.get_cmap("tab20")(np.array(intvector) / (max_label if max_label > 0 else 1))
            # colors[np.array(intvector) < 0] = 0
            # copied_o3dINIT.colors = utility.Vector3dVector(colors[:, :3])
            visualization.draw_geometries([self.newPCL])

            ros_cloud = self.convertCloudFromOpen3dToRos(self.open3d_cloudINIT, frame_id=msg.header.frame_id)
            # print(f"ROS_cloud: {ros_cloud}")
            
            rospy.set_param("/add_PCL", False)
            self.plc_pub.publish(ros_cloud)

            # Try turning PC into Collision Object
            co = self.add_pointcloud_collision_object(ros_cloud, frame_id="world")
            self.sendCO_2_PI(co)
        
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