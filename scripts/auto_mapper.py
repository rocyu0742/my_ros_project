#!/usr/bin/env python3
"""
Auto-Mapper: Otomatik Haritalama Script'i
Robot belirli waypoint'leri gezerek haritayı otomatik oluşturur.
GMapping ile birlikte çalışır.

Kullanım:
1. roslaunch me48b_library_robot mapping_auto.launch
2. Script bitince harita otomatik kaydedilir
"""

import rospy
from geometry_msgs.msg import Twist
import time
import subprocess
import os

class AutoMapper:
    def __init__(self):
        rospy.init_node('auto_mapper')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Comprehensive Exploration Pattern (Lawn Mower + Corners)
        # Format: (linear_x, angular_z, duration_sec)
        self.movements = [
            # Phase 1: Initial 360 scan at spawn
            (0.0, 0.5, 12.0),   # Full rotation
            
            # Phase 2: Go forward, explore right side
            (0.25, 0.0, 8.0),   # Forward 2m
            (0.0, -0.5, 3.14),  # Turn right 90
            (0.25, 0.0, 6.0),   # Forward 1.5m
            (0.0, -0.5, 3.14),  # Turn right 90
            (0.25, 0.0, 8.0),   # Forward 2m
            (0.0, 0.5, 12.0),   # 360 scan
            
            # Phase 3: Continue sweep
            (0.0, -0.5, 3.14),  # Turn right 90
            (0.25, 0.0, 6.0),   # Forward 1.5m
            (0.0, -0.5, 3.14),  # Turn right 90
            (0.25, 0.0, 8.0),   # Forward 2m
            (0.0, 0.5, 12.0),   # 360 scan
            
            # Phase 4: Explore far corners
            (0.0, 0.5, 3.14),   # Turn left 90
            (0.25, 0.0, 10.0),  # Forward 2.5m (far)
            (0.0, 0.5, 12.0),   # 360 scan
            (0.0, 0.5, 3.14),   # Turn left 90
            (0.25, 0.0, 8.0),   # Forward 2m
            (0.0, 0.5, 12.0),   # 360 scan
            
            # Phase 5: Return sweep (opposite direction)
            (0.0, 0.5, 6.28),   # Turn 180
            (0.25, 0.0, 8.0),   # Forward 2m
            (0.0, -0.5, 3.14),  # Turn right 90
            (0.25, 0.0, 10.0),  # Forward 2.5m
            (0.0, 0.5, 12.0),   # 360 scan
            
            # Phase 6: Center exploration
            (0.0, 0.5, 3.14),   # Turn left 90
            (0.25, 0.0, 6.0),   # Forward
            (0.0, 0.5, 12.0),   # Final 360 scan
            
            # Phase 7: Return to origin area
            (-0.2, 0.0, 5.0),   # Backup
            (0.0, 0.5, 12.0),   # Final full rotation
        ]
        
        rospy.loginfo("🗺️ Auto-Mapper başlatıldı!")
        
    def run(self):
        rospy.loginfo("📍 Haritalama başlıyor...")
        
        twist = Twist()
        rate = rospy.Rate(10)
        
        for i, (lin_x, ang_z, duration) in enumerate(self.movements):
            if rospy.is_shutdown():
                break
                
            rospy.loginfo(f"🚗 Adım {i+1}/{len(self.movements)}: lin={lin_x:.1f} ang={ang_z:.1f} dur={duration:.1f}s")
            
            twist.linear.x = lin_x
            twist.angular.z = ang_z
            
            start_time = time.time()
            while time.time() - start_time < duration and not rospy.is_shutdown():
                self.vel_pub.publish(twist)
                rate.sleep()
        
        # Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)
        
        rospy.loginfo("✅ Haritalama tamamlandı! Harita kaydediliyor...")
        
        # Save Map
        self.save_map()
        
    def save_map(self):
        map_path = "/home/rocyu/my_ros_project/maps/library_map"
        
        try:
            # Call map_saver
            result = subprocess.run(
                ["rosrun", "map_server", "map_saver", "-f", map_path],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                rospy.loginfo(f"💾 Harita kaydedildi: {map_path}.yaml")
            else:
                rospy.logerr(f"❌ Kaydetme hatası: {result.stderr}")
                
        except Exception as e:
            rospy.logerr(f"❌ Map saver hatası: {e}")

if __name__ == '__main__':
    try:
        mapper = AutoMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
