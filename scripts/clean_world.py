#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
import time

def clean_world():
    rospy.init_node('clean_world_node')
    
    # 1. Delete Models
    delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    
    # Check args
    parser = argparse.ArgumentParser()
    parser.add_argument('--books', action='store_true', help='Delete only books')
    args, _ = parser.parse_known_args()
    
    if args.books:
        models_to_delete = ["test_book", "book_s1", "book_s2", "book_s3"]
        print("🧹 Cleaning ONLY Books...")
    else:
        models_to_delete = ["station_1", "station_2", "station_3", "test_book", "book_s1", "book_s2", "book_s3"]
        print("🧹 Cleaning World (Stations + Books)...")
        
    for model in models_to_delete:
        try:
            delete_service(model)
            print(f"   Deleted {model}")
        except rospy.ServiceException:
            pass # Maybe it didn't exist

    # Reset Robot & Forklift ONLY if doing full clean
    if not args.books:
        # 2. Reset Robot Position
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        robot_state = ModelState()
        robot_state.model_name = "turtlebot3_burger" # Correct name from Gazebo
        robot_state.pose.position.x = 0.5
        robot_state.pose.position.y = 1.0
        robot_state.pose.position.z = 0.05
        robot_state.pose.orientation.x = 0.0
        robot_state.pose.orientation.y = 0.0
        robot_state.pose.orientation.z = 0.0
        robot_state.pose.orientation.w = 1.0
        
        try:
            set_state(robot_state)
            print("🤖 Robot reset to Home (0.5, 1.0) in Gazebo")
        except rospy.ServiceException as e:
            print(f"⚠️ Could not reset robot: {e}")
        
        # 2b. SYNC AMCL (Navigation) with Gazebo Position
        from geometry_msgs.msg import PoseWithCovarianceStamped
        amcl_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10, latch=True)
        time.sleep(1.0) # Wait for publisher to fully connect
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.pose.pose.position.x = 0.5
        initial_pose.pose.pose.position.y = 1.0
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        # Covariance (low values = high confidence)
        initial_pose.pose.covariance = [0.01] * 36
        
        # Publish multiple times to ensure AMCL receives
        for i in range(5):
            initial_pose.header.stamp = rospy.Time.now()
            amcl_pub.publish(initial_pose)
            time.sleep(0.2)
        
        print("🗺️ AMCL position synced to (0.5, 1.0)")
    
        # 3. Reset Forklift (Publish 0.0 to controller)
        from std_msgs.msg import Float64
        pub_fork = rospy.Publisher('/fork_joint_position_controller/command', Float64, queue_size=1)
        
        # Wait a bit for connection
        time.sleep(1.0) 
        
        # Publish multiple times to ensure it receives
        for _ in range(5):
            pub_fork.publish(0.0)
            time.sleep(0.1)
        
        print("🏗️ Forklift Reset to 0.0")

    print("✨ World Cleaned!")

if __name__ == '__main__':
    import argparse
    clean_world()
