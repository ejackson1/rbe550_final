from centroidSubscriber import centroidsubscriber, moveTo
from moveitClient import MoveitArmClient
import geometry_msgs

def moveTo(pose: tuple):
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
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
    #  pose_goal.position.z = 0.3
        print("Requesting...")
        m.move_arm_EE(pose_goal)

if __name__ == "__main__":
    centroidsubscriber() #initiates Subscriber to look for centroid data, plan, and move to centroid pose

    through_doorway = (x,y,z)

    moveTo(through_doorway) # Robot will plan a path and execute it to the specified position

