#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math

# Lista wszystkich kontrolerów
joint_names = [
    "hip_joint_1", "hip_joint_2", "hip_joint_3",
    "hip_joint_4", "hip_joint_5", "hip_joint_6",
    "knee_joint_1", "knee_joint_2", "knee_joint_3",
    "knee_joint_4", "knee_joint_5", "knee_joint_6",
    "ankle_joint_1", "ankle_joint_2", "ankle_joint_3",
    "ankle_joint_4", "ankle_joint_5", "ankle_joint_6"
]

# Żądane kąty w stopniach
desired_angles_deg = {
    # Biodra: krańcowe 45°, środkowe 0°
    "hip_joint_1": -45,
    "hip_joint_2": 45,
    "hip_joint_3": 0,
    "hip_joint_4": 0,
    "hip_joint_5": 45,
    "hip_joint_6": -45,

    # Kolana i kostki - jak wcześniej
    "knee_joint": -30,
    "ankle_joint": -120
}

def main():
    rospy.init_node("hexapod_mover")
    rospy.loginfo("Czekam na uruchomienie kontrolerów...")
    rospy.sleep(1.5)

    rospy.loginfo("Tworzę publishery...")
    publishers = {}
    for joint in joint_names:
        topic = f"/{joint}_position_controller/command"
        publishers[joint] = rospy.Publisher(topic, Float64, queue_size=10)
    rospy.sleep(1.0)

    rospy.loginfo("Ustawiam robota w pozycję stojącą z wychyleniem bioder...")

    for joint in joint_names:
        if joint.startswith("hip_joint"):
            angle_deg = desired_angles_deg[joint]
        elif joint.startswith("knee_joint"):
            angle_deg = desired_angles_deg["knee_joint"]
        elif joint.startswith("ankle_joint"):
            angle_deg = desired_angles_deg["ankle_joint"]
        else:
            continue

        angle_rad = math.radians(angle_deg)
        publishers[joint].publish(angle_rad)

    rospy.loginfo("Wszystkie komendy wysłane.")
    rospy.sleep(2.0)

if __name__ == "__main__":
    main()
