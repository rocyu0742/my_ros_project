#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetLinkState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import sys

# Robot model name in Gazebo (might need adjustment if spawned differently)
ROBOT_MODEL_NAME = "turtlebot3_burger" # We need to check if it's burger or forklift
FORK_LINK_NAME = "fork_link"

class BookManager:
    def __init__(self):
        rospy.init_node('book_manager')
        
        # Services
        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        self.attached_book = None
        self.offset_z = 0.05 # Height above fork
        
        rospy.Timer(rospy.Duration(0.05), self.update_loop) # 20 Hz
        
        print("📖 Book Manager Started!")
        print("   Use: attach_book('book_red_1') to pick up")
        print("   Use: detach_book() to drop")

    def attach_book(self, book_name):
        self.attached_book = book_name
        print(f"🧲 ATTACHED: {book_name}")

    def detach_book(self):
        if self.attached_book:
            print(f"💨 DETACHED: {self.attached_book}")
            self.attached_book = None
        else:
            print("⚠️ No book attached to detach.")

    def update_loop(self, event):
        if self.attached_book is None:
            return

        try:
            # 1. Get Fork Position
            # Note: Link name format usually "model_name::link_name"
            # We try standard naming first
            
            # Since we cleared the world, robot name might be just "turtlebot3_burger"
            # or what we defined in spawn. Let's try to construct it.
            # We can check available links if it fails, but let's guess first.
            
            fork_link_full_name = "turtlebot3_burger::fork_link" 
            
            link_state = self.get_link_state(link_name=fork_link_full_name, reference_frame="world")
            
            if not link_state.success:
                # Try creating warning only once effectively or debugging
                # rospy.logwarn_throttle(1, f"Could not get state of {fork_link_full_name}")
                return

            fork_pose = link_state.link_state.pose

            # 2. Calculate Book Position (Attached to Fork)
            book_pose = Pose()
            book_pose.position.x = fork_pose.position.x
            book_pose.position.y = fork_pose.position.y
            book_pose.position.z = fork_pose.position.z + self.offset_z
            book_pose.orientation = fork_pose.orientation # Align with fork

            # 3. Set Book Position
            state_msg = ModelState()
            state_msg.model_name = self.attached_book
            state_msg.pose = book_pose
            state_msg.reference_frame = "world"

            resp = self.set_model_state(state_msg)
            # if not resp.success:
            #     pass 

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

# Simple interactive CLI for testing
if __name__ == '__main__':
    manager = BookManager()
    
    # Keep main thread alive and allow simple input
    while not rospy.is_shutdown():
        try:
            cmd = input("Command (attach <name> / detach / quit): ").strip().split()
            if not cmd: continue
            
            if cmd[0] == 'attach' and len(cmd) > 1:
                manager.attach_book(cmd[1])
            elif cmd[0] == 'detach':
                manager.detach_book()
            elif cmd[0] == 'quit':
                break
            else:
                print("Invalid command.")
        except EOFError:
            break
