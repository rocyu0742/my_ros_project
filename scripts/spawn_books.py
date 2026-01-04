#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import rospkg

def spawn_book(name, model_dir, x, y, z):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Load SDF
        rospack = rospkg.RosPack()
        path = rospack.get_path('me48b_library_robot') + f"/models/{model_dir}/model.sdf"
        
        with open(path, "r") as f:
            sdf_xml = f.read()
            
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Rotation (upright)
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        
        spawn_sdf(name, sdf_xml, "/", pose, "world")
        print(f"Spawned {name} at ({x}, {y}, {z})")
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('book_spawner')
    
    # Spawn books directly in front of the robot (assuming robot starts at 0.0, 0.0)
    # The robot is at (0.5, 1.0), so we spawn at x=1.0 (0.5m ahead)
    
    # Robot is stuck at (0.5, 1.0). Spawn books very close in front of it.
    
    # Book 1: Red (Shelf A) - Center
    spawn_book("book_red_1", "book_red", 0.7, 1.0, 0.4) 
    
    # Book 2: Blue (Shelf B) - Slightly Left
    spawn_book("book_blue_1", "book_blue", 0.7, 1.1, 0.4) 

    # Book 3: Green (Shelf C) - Slightly Right
    spawn_book("book_green_1", "book_green", 0.7, 0.9, 0.4)
