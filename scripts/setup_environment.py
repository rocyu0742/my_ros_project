#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, GetModelState, DeleteModel
from geometry_msgs.msg import Pose
import rospkg
import time
import os
import argparse
import sys

from tf.transformations import quaternion_from_euler

def spawn_model(model_name, model_pkg_path, x, y, z, R=0, P=0, Y=0):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        with open(model_pkg_path, 'r') as f:
            model_xml = f.read()
            
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z
        
        q = quaternion_from_euler(R, P, Y)
        initial_pose.orientation.x = q[0]
        initial_pose.orientation.y = q[1]
        initial_pose.orientation.z = q[2]
        initial_pose.orientation.w = q[3]
        
        spawn(model_name, model_xml, "", initial_pose, "world")
        print(f"✅ Spawned {model_name} at ({x:.2f}, {y:.2f}, {z:.2f})")
        
    except rospy.ServiceException as e:
        print(f"❌ Service call failed: {e}")

def check_station_exists(station_id):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(f"station_{station_id}", "")
        return resp.success
    except rospy.ServiceException:
        return False

def spawn_book_at_station(station_id, pkg_path, color="red"):
    if not check_station_exists(station_id):
        print(f"❌ Cannot spawn book: station_{station_id} does not exist!")
        return

    # Valid Coordinates (Z ~ 1.03m means on top of existing desks)
    # S1: InfoDesk (Facing East-ish)
    S1 = (-0.684057, 0.795968, 1.031435)
    # S2: DeskB (Far East)
    S2 = (5.378602, 0.405931, 0.908775)
    # S3: DeskA (North West) - Height matched to S1
    # S3: DeskA (North West) - Height matched to S1
    S3 = (-2.795135, 2.816197, 1.031435)

    coords = None
    if station_id == 1: coords = S1
    elif station_id == 2: coords = S2
    elif station_id == 3: coords = S3
    
    # Select Book Model based on Color
    model_name_map = {
        "red": "book_red",
        "green": "book_green", 
        "blue": "book_blue"
    }
    
    if color not in model_name_map:
        print(f"⚠️ Unknown color '{color}', defaulting to red.")
        color = "red"
        
    book_pkg = model_name_map[color]
    
    if coords:
        book_sdf = f"{pkg_path}/models/{book_pkg}/model.sdf"
        name = f"book_s{station_id}"
        
        # Rotations based on Robot Approach
        # S1: Robot from West, Looking East. Book Face West? No wait.
        # Robot S1 Approach: From East (0, 0.8), Looking West (-X). Book should face East (+X). Default (0) is fine.
        
        # S2: 180 degrees (3.14)
        # S3: -90 degrees (-1.57) - Clockwise 90
        
        yaw = 0
        if station_id == 2: yaw = 3.14
        elif station_id == 3: yaw = -1.57
        
        # 4. ROBUST DELETE (Wait until truly gone)
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            
            # Check if exists
            if get_state(name, "").success:
                print(f"🗑️ Deleting old {name}...")
                delete(name)
                
                # Wait for removal (Max 2s)
                for _ in range(20):
                    if not get_state(name, "").success:
                        print(f"✅ Old {name} removed.")
                        break
                    time.sleep(0.1)
                else:
                    print(f"⚠️ Delete Timed Out, attempting spawn anyway...")
                    
        except Exception as e:
            print(f"⚠️ Clean Error: {e}")
        
        spawn_model(name, book_sdf, coords[0], coords[1], coords[2] + 0.05, Y=yaw) 

def spawn_stations(pkg_path):
    S1 = (-0.684057, 0.795968, 1.031435)
    S2 = (5.378602, 0.405931, 0.908775)
    # S3: DeskA (North West) - Height matched to S1
    S3 = (-2.795135, 2.816197, 1.031435)
    
    station_sdf = f"{pkg_path}/models/dropoff_station/model.sdf"
    spawn_model("station_1", station_sdf, S1[0], S1[1], S1[2]) 
    spawn_model("station_2", station_sdf, S2[0], S2[1], S2[2])
    spawn_model("station_3", station_sdf, S3[0], S3[1], S3[2])

if __name__ == '__main__':
    rospy.init_node('setup_environment')
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('me48b_library_robot')
    
    parser = argparse.ArgumentParser(description='Manage Library Environment')
    parser.add_argument('--stations', action='store_true', help='Spawn dropoff stations')
    parser.add_argument('--book', type=int, choices=[1, 2, 3], help='Spawn book at station ID')
    parser.add_argument('--color', type=str, default='red', choices=['red', 'green', 'blue'], help='Color of the book (red, green, blue)')
    parser.add_argument('--all', action='store_true', help='Spawn everything (Default)')
    
    args, unknown = parser.parse_known_args()
    
    # helper for "python3 setup_environment.py" -> spawn all
    if not args.stations and not args.book:
        args.all = True

    if args.stations or args.all:
        spawn_stations(pkg_path)
        
    if args.book:
        print(f"DEBUG: Spawning Book at S{args.book} with Color: {args.color}")
        spawn_book_at_station(args.book, pkg_path, args.color)
    elif args.all:
        # Default behavior: Spawn book at S1 only
        spawn_book_at_station(1, pkg_path, "red")
    
    print("\n🌍 Environment Update Complete!")
