#!/usr/bin/env python3
import rospy
import time

def toggle_edge_availability():
    rospy.init_node("edge_toggle_controller")

    durations = [
        ("ON", 30),   # Edge available for 30 sec
        ("OFF", 30),  # Edge unavailable for 30 sec
        ("ON", 30),
        ("OFF", 30),
    ]

    rospy.loginfo("Starting edge availability experiment...")

    for state, duration in durations:
        if state == "ON":
            rospy.set_param('/experiment/edge_override', True)
            rospy.loginfo(f"[EXPERIMENT] Edge ENABLED for {duration}s")
        else:
            rospy.set_param('/experiment/edge_override', False)
            rospy.loginfo(f"[EXPERIMENT] Edge DISABLED for {duration}s")

        start_time = time.time()

        while time.time() - start_time < duration:
            rospy.sleep(1)

    rospy.loginfo("Experiment completed!")

if __name__ == "__main__":
    try:
        toggle_edge_availability()
    except rospy.ROSInterruptException:
        pass