#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('forklift_tester')
    pub = rospy.Publisher('/fork_joint_position_controller/command', Float64, queue_size=10)
    
    print("Forklift Kontrolü Başladı!")
    print("1: Yukarı Kaldır (15cm)")
    print("0: Aşağı İndir (0cm)")
    print("q: Çıkış")
    
    while not rospy.is_shutdown():
        cmd = input("Komut girin (1/0/q): ")
        if cmd == '1':
            pub.publish(0.15) # 15 cm yukarı
            print("⬆️ Yukarı kaldırılıyor...")
        elif cmd == '0':
            pub.publish(0.0)  # 0 cm (zemin)
            print("⬇️ Aşağı indiriliyor...")
        elif cmd == 'q':
            break

if __name__ == '__main__':
    main()
