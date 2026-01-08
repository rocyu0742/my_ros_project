#!/usr/bin/env python3

import tkinter as tk
import tkinter.ttk as ttk
from tkinter import scrolledtext
import subprocess
import os
import signal
import rospkg
import rospy
from geometry_msgs.msg import Vector3

class RobotControlPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Library Robot Control Center")
        self.root.geometry("500x480")
        
        # Get path FIRST
        try:
            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('me48b_library_robot')
        except:
            self.pkg_path = "/home/rocyu/my_ros_project"

        # Env setup
        self.env = os.environ.copy()
        if "ROS_PACKAGE_PATH" not in self.env:
             self.env["ROS_PACKAGE_PATH"] = "/opt/ros/noetic/share"
        self.env["ROS_PACKAGE_PATH"] += f":{self.pkg_path}/.."

        self.processes = []
        
        # --- UI Elements (Created in __init__ to persist) ---
        
        # Header
        lbl = tk.Label(root, text="ME48B Mission Control", font=("Arial", 14, "bold"))
        lbl.pack(pady=10)

        # Buttons Frame
        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=10)

        # 1. World & Setup
        btn_world = tk.Button(btn_frame, text="🌍 LAUNCH WORLD", command=self.run_world, bg="#ffffcc", font=("Arial", 9, "bold"))
        btn_world.grid(row=0, column=0, padx=5, pady=5)

        btn_clean = tk.Button(btn_frame, text="🧹 Clean ALL", command=self.run_clean_all, bg="#ffcccc")
        btn_clean.grid(row=0, column=1, padx=5, pady=5)

        btn_setup = tk.Button(btn_frame, text="🛠️ Setup Stations", command=self.run_setup_stations, bg="#ccffcc")
        btn_setup.grid(row=0, column=2, padx=5, pady=5)
        


        # --- SIMULATION SPEED CONTROL ---
        sim_frame = tk.LabelFrame(btn_frame, text="⏱️ Sim Speed", font=("Arial", 8))
        sim_frame.grid(row=1, column=1, columnspan=2, sticky="we", padx=5)
        
        self.lbl_speed = tk.Label(sim_frame, text="1.0x")
        self.lbl_speed.pack(side=tk.LEFT, padx=5)
        
        # Scale from 0.1x to 10.0x
        self.scale_speed = tk.Scale(sim_frame, from_=0.1, to=10.0, orient=tk.HORIZONTAL, resolution=0.1, showvalue=0, command=self.update_sim_speed)
        self.scale_speed.set(1.0)
        self.scale_speed.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # 2. Book Controls (Shifted down to make room for AUTO-MAP)
        lbl_col = tk.Label(btn_frame, text="Select Color:")
        lbl_col.grid(row=2, column=0, pady=5)
        
        self.combo_color = ttk.Combobox(btn_frame, values=["red", "green", "blue"], width=8, state="readonly")
        self.combo_color.current(0) # Default to first item (red)
        self.combo_color.grid(row=2, column=1, pady=5)
        
        # Spacer
        tk.Label(btn_frame, text="").grid(row=3, column=0)

        btn_book1 = tk.Button(btn_frame, text="📚 Spawn S1", command=lambda: self.run_spawn_book(1), bg="#e6f7ff")
        btn_book1.grid(row=4, column=0, padx=2, pady=5)
        
        btn_book2 = tk.Button(btn_frame, text="📚 Spawn S2", command=lambda: self.run_spawn_book(2), bg="#e6f7ff")
        btn_book2.grid(row=4, column=1, padx=2, pady=5)
        
        btn_book3 = tk.Button(btn_frame, text="📚 Spawn S3", command=lambda: self.run_spawn_book(3), bg="#e6f7ff")
        btn_book3.grid(row=4, column=2, padx=2, pady=5)
        
        btn_clear_books = tk.Button(btn_frame, text="🗑️ Clear Books", command=self.run_clear_books, bg="#ffe6e6")
        btn_clear_books.grid(row=5, column=0, columnspan=3, pady=5, sticky="we")

        # 5. Localization
        btn_reset_loc = tk.Button(btn_frame, text="📍 FIX POSITION (0.5, 1.0)", command=self.run_reset_loc, bg="#ccf2ff")
        btn_reset_loc.grid(row=6, column=0, columnspan=3, padx=5, pady=2, sticky="we")

        # 6. Mission
        btn_mission = tk.Button(btn_frame, text="🚀 START MISSION", command=self.run_mission, bg="#ffffcc", font=("Arial", 10, "bold"))
        btn_mission.grid(row=7, column=0, columnspan=3, padx=5, pady=10)

        # 6. Stop
        btn_stop = tk.Button(root, text="🛑 STOP ALL / RESET", command=self.stop_all, bg="red", fg="white")
        btn_stop.pack(pady=5)

        # Log Area
        self.log_area = scrolledtext.ScrolledText(root, height=10)
        self.log_area.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        
        self.log("✅ Control Panel Ready. Full System must be running.")
        
        # ROS Init (Lazy - Schedule AFTER UI is ready)
        self.ros_connected = False
        self.root.after(1000, self.connect_ros)
    
    def connect_ros(self):
        if self.ros_connected: return
        try:
            import rosgraph
            if rosgraph.is_master_online():
                rospy.init_node('control_panel', anonymous=True, disable_signals=True)
                self.ros_connected = True
                self.log("✅ Connected to ROS Master!")
            else:
                self.root.after(2000, self.connect_ros)
        except: 
            self.root.after(2000, self.connect_ros)

    def log(self, msg):
        self.log_area.insert(tk.END, msg + "\n")
        self.log_area.see(tk.END)

    def run_world(self):
        self.log("--> 🌍 LAUNCHING FULL SYSTEM (World + Robot + Nav)...")
        # Launch using pkg path relative launch file
        cmd = f"xterm -e 'roslaunch {self.pkg_path}/launch/full_system.launch'"
        try:
             subprocess.Popen(cmd, shell=True, env=self.env)
        except:
             self.log("⚠️ Failed to launch xterm for world.")

    def run_clean_all(self):
        self.log("--> Cleaning Everything (Reset)...")
        subprocess.Popen(["python3", f"{self.pkg_path}/scripts/clean_world.py"], env=self.env)

    def run_clear_books(self):
        self.log("--> Cleaning Only Books...")
        subprocess.Popen(["python3", f"{self.pkg_path}/scripts/clean_world.py", "--books"], env=self.env)
        
    def run_setup_stations(self):
        self.log("--> Setting up Stations...")
        subprocess.Popen(["python3", f"{self.pkg_path}/scripts/setup_environment.py", "--stations"], env=self.env)



    def update_sim_speed(self, val):
        try:
            speed = float(val)
            self.lbl_speed.config(text=f"{speed:.1f}x")
            
            # Default Physics:
            # step_size = 0.001 (1ms)
            # update_rate = 1000 (1ms * 1000 = 1s simulated / 1s real)
            
            # To speed up 2x:
            # update_rate = 2000
            
            base_rate = 1000
            new_rate = int(base_rate * speed)
            
            # Run gz physics command
            # gz physics -u 2000
            cmd = ["gz", "physics", "-u", str(new_rate)]
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
        except Exception as e:
            print(f"Error setting speed: {e}")

    def run_spawn_book(self, sid):
        # Direct access is safer than StringVar if binding fails
        color = self.combo_color.get() 
        
        self.log(f"--> [DEBUG] GUI Color Selected: '{color}'")
        print(f"--> [DEBUG] Terminal - GUI Value: '{color}'")
        
        cmd = ["python3", f"{self.pkg_path}/scripts/setup_environment.py", "--book", str(sid), "--color", color]
        self.log(f"--> [DEBUG] Running: {' '.join(cmd)}")
        
        subprocess.Popen(cmd, env=self.env)

    def run_qr(self):
        self.log("--> Starting QR Reader (Background)...")
        p = subprocess.Popen(["python3", f"{self.pkg_path}/scripts/qr_reader.py"], env=self.env)
        self.processes.append(p)

    def run_reset_loc(self):
        self.log("--> 📍 Resetting AMCL Localization to (0.5, 1.0)...")
        # Publish initial pose to AMCL matchin Spawn Location (0.5, 1.0)
        cmd = [
            "rostopic", "pub", "-1", "/initialpose", "geometry_msgs/PoseWithCovarianceStamped",
            "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.5, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}"
        ]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
    def run_mission(self):
        # Optional: Auto-reset loc on mission start? No, let user do it.
        # Auto-start QR Reader

        self.run_qr()
        
        self.log("--> 🚀 LAUNCHING MISSION! Check Terminal for details...")
        # Running in a new terminal window so user can see live logs clearly
        cmd = f"xterm -e 'python3 {self.pkg_path}/scripts/library_mission.py; read'"
        try:
            subprocess.Popen(cmd, shell=True, env=self.env)
        except:
             self.log("⚠️ xterm not found, running invisible...")
             p = subprocess.Popen(["python3", f"{self.pkg_path}/scripts/library_mission.py"], env=self.env)
             self.processes.append(p)

             p = subprocess.Popen(["python3", f"{self.pkg_path}/scripts/library_mission.py"], env=self.env)
             self.processes.append(p)

    def stop_all(self):
        self.log("🛑 Stopping initiated processes...")
        for p in self.processes:
            p.terminate()
        self.processes = []
        subprocess.call(["pkill", "-f", "library_mission.py"])
        subprocess.call(["pkill", "-f", "qr_reader.py"])
        self.log("✅ All stopped.")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlPanel(root)
    root.mainloop()
