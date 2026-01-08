#!/usr/bin/env python3

import rospy
import actionlib
import tf
import time
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

class ContinuousMission:
    def __init__(self):
        rospy.init_node('continuous_library_mission')
        
        # --- Config ---
        # Central Waiting Spot
        self.LOC_IDLE = (0.75, 0.85, 0.0, 1.0) 
        
        # Pickup Locations: (x, y, z_orient, w_orient)
        # Strategy: Approach from East (+X) side, Face West (-X)
        self.STATIONS = {
            1: {'pickup': (0.0, 0.8, 1.0, 0.0),    'book_model': 'book_s1'},
            # S2 Adjusted: Moved to 4.6 (Sweet spot between stuck and too far)
            2: {'pickup': (4.6, 0.4, 0.0, 1.0),    'book_model': 'book_s2'},
            # S3 Adjusted: Table at (-2.8, 2.8). Robot at (-2.8, 2.1). Face North (+Y)
            # S3 Adjusted: Robot at (-2.8, 2.1). Face West (180 deg) as requested
            3: {'pickup': (-2.8, 2.1, 1.0, 0.0),   'book_model': 'book_s3'}
        }
        
        # Dropoff Locations (Shelves)
        # Dropoff Locations (Shelves)
        self.SHELVES = {
            # Shelf A: East Side. Approach Point (0.8). Blind Docking will drive to 1.45.
            "SHELF_A": (0.8, -0.5, 0.0, 1.0),
            # Shelf B: West Side. Approach (-1.9). Face West (1.0, 0.0). Blind Dock -> -2.55
            "SHELF_B": (-1.9, -0.5, 1.0, 0.0),
            # Shelf C: South Side. Reverted to direct (-3.0).
            "SHELF_C": (0.0, -3.0, 1.0, 0.0)
        }
        
        # --- Clients ---
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("⏳ Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("✅ move_base connected!")
        
        self.fork_pub = rospy.Publisher('/fork_joint_position_controller/command', Float64, queue_size=1)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        self.qr_sub = rospy.Subscriber('/detected_qr', String, self.qr_callback)
        self.metric_sub = rospy.Subscriber('/detected_qr_metric', Point, self.metric_callback) # Metric Sub
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback) # Joint Sub
        self.model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # --- State ---
        self.latest_qr = None
        self.qr_metric = None # (x, y, area)
        self.carrying_book = False
        self.real_fork_height = 0.1 # Initialize

    def joint_callback(self, msg):
        if "fork_joint" in msg.name:
            try:
                idx = msg.name.index("fork_joint")
                self.real_fork_height = msg.position[idx]
            except: pass

    def get_robot_pose(self):
        try:
            resp = self.model_state_service('turtlebot3_burger', '')
            return resp.pose.position
        except:
            return Point(0,0,0)

    def check_for_books(self):
        """Returns the ID of the closest station with a book, or None."""
        robot_pos = self.get_robot_pose()
        closest_station = None
        min_dist = 999.0
        
        potential_books = {
            1: ['book_s1', 'test_book'],
            2: ['book_s2'],
            3: ['book_s3']
        }
        
        for sid, models in potential_books.items():
            found = False
            for model in models:
                res = self.model_state_service(model, '')
                if res.success:
                    # Check Z to ensure it's on the table (not on ground)
                    if res.pose.position.z > 0.5:
                        found = True
                        break
            
            if found:
                # Calculate distance to this station's pickup point
                px, py, _, _ = self.STATIONS[sid]['pickup']
                dist = math.sqrt((px - robot_pos.x)**2 + (py - robot_pos.y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_station = sid
                    
        return closest_station

    def move_fork(self, height, wait=True):
        rospy.loginfo(f"🏗️ Moving Fork to {height:.2f}m")
        self.fork_pub.publish(height)
        if wait:
            time.sleep(2.0)

    def search_vertical_for_qr(self):
        """ Lifts fork incrementally until QR is seen """
        rospy.loginfo("🧐 Searching for QR vertically...")
        
        current_h = 0.6 # Start near table height
        step = 0.05
        max_h = 1.3 # Scan above the table 
        
        self.move_fork(current_h, wait=True)
        
        while not rospy.is_shutdown():
            if self.latest_qr:
                rospy.loginfo(f"✅ QR Found at height {current_h:.2f}m!")
                self.current_fork_height = current_h # Store state
                return True
                
            current_h += step
            if current_h > max_h:
                rospy.logwarn("❌ Reached max height, QR not found.")
                return False
                
            self.fork_pub.publish(current_h)
            time.sleep(0.5) 
            
        return False

    def move_to(self, x, y, z, w):
        rospy.loginfo(f"🚗 Moving to: x={x:.2f}, y={y:.2f}...")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def backup_maneuver(self):
        rospy.loginfo("🔙 Backing up safely...")
        twist = Twist()
        twist.linear.x = -0.2
        for _ in range(15):
            self.vel_pub.publish(twist)
            time.sleep(0.1)
        self.vel_pub.publish(Twist())
        time.sleep(0.5)

    def magnet_update(self, event):
        """Timer callback to keep book attached to fork"""
        if self.carrying_book and self.attached_book_name:
            # Get Robot Pose
            r_res = self.model_state_service("turtlebot3_burger", "")
            if not r_res.success: return
            
            p = r_res.pose.position
            o = r_res.pose.orientation
            
            # Quaternion to Euler
            q = [o.x, o.y, o.z, o.w]
            _, _, yaw = tf.transformations.euler_from_quaternion(q)
            
            # Dynamic Z based on REAL Fork Height (from JointState)
            current_z = getattr(self, 'real_fork_height', 0.1)
            
            # Book Position:
            # Robot XY + Offset (Forward)
            # Robot Z (0) + Fork Height + Book Offset
            
            new_state = ModelState()
            new_state.model_name = self.attached_book_name
            new_state.pose.position.x = p.x + 0.20 * math.cos(yaw) # Slightly closer (0.20 instead of 0.25)
            new_state.pose.position.y = p.y + 0.20 * math.sin(yaw)
            new_state.pose.position.z = current_z + 0.05 # Just above fork
            new_state.pose.orientation = o
            new_state.reference_frame = "world"
            
            try:
                resp = self.set_model_state(new_state)
                if not resp.success:
                    rospy.logwarn_throttle(1.0, f"🧲 Magnet Fail: {resp.status_message}")
            except Exception as e:
                rospy.logerr_throttle(1.0, f"🧲 Magnet Error: {e}")

    def qr_callback(self, msg):
        self.latest_qr = msg.data

    def metric_callback(self, msg):
        self.qr_metric = msg # x=cx, y=cy, z=area

    def visual_servoing_approach(self):
        rospy.loginfo("👁️ STARTING 3D VISUAL SERVOING (X + Y-Height)...")
        
        twist = Twist()
        rate = rospy.Rate(10)
        
        # Config
        TARGET_AREA = 25000 
        # Config
        TARGET_AREA = 25000 
        CENTER_X = 320     
        CENTER_Y = 185     # Target Y (185 = Slightly higher than 150 to clear station lip)
        
        Kp_ang = 0.004     # Rotation Gain
        Kp_fork = 0.0005   # Vertical Gain (Height per pixel error)
        
        start_time = time.time()
        locked = False
        
        # Track current height (assume we start at the height where we found it)
        # We need to know where we are. search_vertical_for_qr leaves us at a height.
        # But we don't store it in a class var. Let's start with a safe guess or read it?
        # Better: search_vertical_for_qr should set self.current_fork_height
        if not hasattr(self, 'current_fork_height'):
            self.current_fork_height = 0.5 # Fallback
            
        while time.time() - start_time < 25.0:
            if not self.qr_metric:
                rospy.logwarn("   Waiting for visual lock...")
                time.sleep(0.2)
                continue
                
            cx = self.qr_metric.x
            cy = self.qr_metric.y
            area = self.qr_metric.z
            
            error_x = CENTER_X - cx
            error_y = CENTER_Y - cy # If cy < 240 (Top), error > 0. Need Up.
            
            # --- Vertical Servoing (Fork Height) ---
            # If QR is at top (cy small), error_y is positive. 
            # Camera Up -> Object moves Down. 
            # So Positive Error -> Increase Height.
            
            delta_h = Kp_fork * error_y
            self.current_fork_height += delta_h
            
            # Clamp Safety
            self.current_fork_height = max(0.1, min(1.3, self.current_fork_height))
            self.fork_pub.publish(self.current_fork_height)
            
            # --- Horizontal Servoing (Robot Rotation) ---
            if abs(error_x) > 20: # Slightly relaxed strictness for 3D
                twist.linear.x = 0.0
                twist.angular.z = Kp_ang * error_x
                rospy.loginfo(f"   🔄 Centering X:{error_x} Y:{error_y} H:{self.current_fork_height:.2f}")
            else:
                # Aligned -> Approach
                twist.angular.z = Kp_ang * error_x 
                
                if area < TARGET_AREA:
                    twist.linear.x = 0.04 
                    rospy.loginfo(f"   ⬆️ Approaching... Area: {area:.0f} H:{self.current_fork_height:.2f}")
                else:
                    # Area is good, but are we aligned?
                    if abs(error_x) < 30 and abs(error_y) < 30:
                        rospy.loginfo(f"✅ Target Reached & Aligned! Area: {area} X_err:{error_x} Y_err:{error_y}")
                        locked = True
                        break
                    else:
                        # Close enough but not aligned yet. Stop moving forward, just adjust.
                        twist.linear.x = 0.0
                        twist.angular.z = Kp_ang * error_x 
                        rospy.logwarn_throttle(0.5, f"⚠️ Close but correcting alignment... X:{error_x} Y:{error_y}")
                
            self.vel_pub.publish(twist)
            rate.sleep()
        
        self.vel_pub.publish(Twist())
        rospy.loginfo("👁️ Visual Approach Complete.")
        return locked

    def run_loop(self):
        # Initial Reset
        rospy.loginfo("🔄 Initializing Robot State...")
        self.move_fork(0.05)
        
        # Start Magnet Timer
        self.magnet_timer = rospy.Timer(rospy.Duration(0.1), self.magnet_update)
        rospy.loginfo("♾️ STARTING CONTINUOUS MISSION LOOP")
        
        while not rospy.is_shutdown():
            if not self.carrying_book:
                # --- IDLE / SEARCH STATE ---
                target_station = self.check_for_books() # Check all sources
                
                if target_station:
                    rospy.loginfo(f"📚 Book found at Station {target_station}! Going to pickup...")
                    self.perform_pickup(target_station)
                else:
                    rospy.loginfo("💤 No books found. Moving to IDLE...")
                    # Go to idle if far
                    cur = self.get_robot_pose()
                    dist_to_idle = math.hypot(cur.x - self.LOC_IDLE[0], cur.y - self.LOC_IDLE[1])
                    
                    if dist_to_idle > 0.2:
                        self.move_to(*self.LOC_IDLE)
                    else:
                        time.sleep(1.0) # Just wait
            
            else:
                # --- DROPOFF STATE ---
                self.perform_dropoff()

    def perform_pickup(self, station_id):
        coords = self.STATIONS[station_id]['pickup']
        self.attached_book_name = self.STATIONS[station_id]['book_model'] # Track Model
        
        rospy.loginfo(f"📚 Going to Pickup {self.attached_book_name}...")
        self.move_to(*coords)
        
        # Sequence
        self.move_fork(0.0) 
        self.latest_qr = None
        self.qr_metric = None # Reset
        
        # 1. Dynamic Search (Lift until seen)
        if self.search_vertical_for_qr():
            # QR is visible now.
            time.sleep(0.5) # Stabilize
        
            # 2. Visual Approach (3D)
            if self.visual_servoing_approach():
                 # 3. Lift to Attach (Now we depend on visual height?)
                 # Actually, visual servoing might leave us at a perfect height!
                 # Book is usually centered.
                 # Let's just ensure we are under it? 
                 # If we centered the QR, the fork (cam) is aligned with QR.
                 # QR is usually ON the book spine.
                 # Fork is BELOW camera? No, camera is ON fork.
                 # So fork might hit the book?
                 # We need to adjust slightly? 
                 # Let's assume standard offset or just lift.
                 
                 # 4. Final Insertion
                 # We trust visual servoing height! No hardcoded 1.03m.
                 
                 twist = Twist()
                 twist.linear.x = 0.1
                 for _ in range(15): # 1.5s * 0.1 = 15cm
                     self.vel_pub.publish(twist)
                     time.sleep(0.1)
                 self.vel_pub.publish(Twist())
                 
                 # 5. ATTACH (Magnet Logic ON)
                 rospy.loginfo("🧲 MAGNET ON!")
                 self.carrying_book = True
                 
                 # 6. Lift Relative to Pickup Height
                 lift_height = self.current_fork_height + 0.15
                 self.move_fork(lift_height)
                 
                 # 7. Backup
                 self.backup_maneuver()
                 
                 # 8. Lower to Carry (But keep high enough to clear Lidar!)
                 self.move_fork(0.4)
        else:
             rospy.logerr("❌ Visual Servoing Failed. Aborting Pickup.")

    def blind_move(self, distance, speed):
        """Moves robot open-loop (without navigation) for specific distance"""
        rospy.loginfo(f"🙈 Blind Move: Dist={distance}, Speed={speed}")
        twist = Twist()
        twist.linear.x = speed
        
        duration = distance / abs(speed)
        end_time = time.time() + duration
        
        rate = rospy.Rate(10)
        while time.time() < end_time and not rospy.is_shutdown():
            self.vel_pub.publish(twist)
            rate.sleep()
        
        self.vel_pub.publish(Twist()) # Stop

    def perform_dropoff(self):
        # Decide shelf based on QR
        shelf_target = self.SHELVES["SHELF_A"] # Default
        if self.latest_qr in self.SHELVES:
            shelf_target = self.SHELVES[self.latest_qr]
            rospy.loginfo(f"✅ Scanning QR Result: {self.latest_qr} -> Going to {self.latest_qr}")
        elif self.latest_qr:
             rospy.loginfo(f"⚠️ Unknown QR: {self.latest_qr} -> Defaulting to Shelf A")
             
        rospy.loginfo(f"🚚 Delivering to {self.latest_qr}...")
        self.move_to(*shelf_target)
        
        if self.latest_qr == "SHELF_C":
            # --- NORMAL DROPOFF (Direct Nav) ---
            rospy.loginfo("⚓ Docking to Shelf C (Standard Navigation)...")
            # Already at target (-3.0)
            
            # Dropoff Sequence
            self.move_fork(0.9) 
            rospy.loginfo("💨 MAGNET OFF - Detaching...")
            self.carrying_book = False 
            time.sleep(0.5)
            
            # Backup (Standard)
            self.backup_maneuver()
            
        else:
            # --- BLIND STOCKING (Approach + Push) ---
            # Used for Shelf A and B to avoid collision costmap issues
            rospy.loginfo("⚓ Docking to Shelf (Blind Approach)...")
            self.blind_move(0.65, 0.15) # Move forward 65cm
            
            # Dropoff Sequence
            self.move_fork(0.9) 
            rospy.loginfo("💨 MAGNET OFF - Detaching...")
            self.carrying_book = False 
            time.sleep(0.5)
            
            # Backup (Blind)
            rospy.loginfo("🔙 Backing up from Shelf...")
            self.blind_move(0.8, -0.2) # Backup 80cm
        
        # Reset Fork
        self.move_fork(0.05)

if __name__ == '__main__':
    mission = ContinuousMission()
    try:
        mission.run_loop()
    except rospy.ROSInterruptException:
        pass
