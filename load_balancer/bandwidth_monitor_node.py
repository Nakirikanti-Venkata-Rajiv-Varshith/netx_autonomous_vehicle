#!/usr/bin/env python3
import rospy
import threading
import speedtest
from std_msgs.msg import Float32MultiArray

class BandwidthMonitorNode:
    def __init__(self):
        rospy.init_node("bandwidth_monitor_node")

        self.pub = rospy.Publisher("/network/bandwidth", Float32MultiArray, queue_size=1)

        self.download = 0.0
        self.upload = 0.0

        self.measure_thread = threading.Thread(target=self.measure_bandwidth)
        self.measure_thread.daemon = True
        self.measure_thread.start()

        rospy.loginfo("Bandwidth Monitor Node started.")
        rospy.spin()

    def measure_bandwidth(self):
        while not rospy.is_shutdown():
            try:
                st = speedtest.Speedtest()
                st.get_best_server()
                self.download = st.download() / 1e6  # Mbps
                self.upload = st.upload() / 1e6

                msg = Float32MultiArray()
                msg.data = [round(self.upload, 2), round(self.download, 2)]
                self.pub.publish(msg)

                rospy.loginfo(f"[Bandwidth] Upload: {self.upload:.2f} Mbps | Download: {self.download:.2f} Mbps")

            except Exception as e:
                rospy.logwarn(f"Speedtest error: {e}")

            rospy.sleep(10)  # Run speedtest every 10 seconds


if __name__ == "__main__":
    BandwidthMonitorNode()
