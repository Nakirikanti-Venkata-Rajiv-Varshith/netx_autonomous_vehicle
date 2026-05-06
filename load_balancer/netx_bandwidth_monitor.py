#!/usr/bin/env python3
import collections
import socket
import time

import psutil
import rospy
from std_msgs.msg import Float32MultiArray



class BandwidthMonitorNode:
    def __init__(self):
        rospy.init_node("bandwidth_monitor_node_setb")

        self.interface = rospy.get_param("~interface", "wlan0")
        self.edge_host = rospy.get_param("~edge_host", "192.168.20.16")
        self.sample_period = rospy.get_param("~sample_period", 1.0)
        self.rtt_window = rospy.get_param("~rtt_window", 12)

        self.pub = rospy.Publisher("/network/bandwidth", Float32MultiArray, queue_size=1)

        self.last_counters = None
        self.last_sample_time = None
        self.rtt_samples = collections.deque(maxlen=self.rtt_window)

        rospy.loginfo("Bandwidth Monitor Node started in passive mode.")
        self.measure_bandwidth()

    def _sample_bandwidth(self):
        counters = psutil.net_io_counters(pernic=True).get(self.interface)
        now = time.time()
        if counters is None:
            return 0.0, 0.0

        if self.last_counters is None:
            self.last_counters = counters
            self.last_sample_time = now
            return 0.0, 0.0

        dt = max(now - self.last_sample_time, 1e-3)
        upload = ((counters.bytes_sent - self.last_counters.bytes_sent) * 8.0) / (dt * 1e6)
        download = ((counters.bytes_recv - self.last_counters.bytes_recv) * 8.0) / (dt * 1e6)

        self.last_counters = counters
        self.last_sample_time = now
        return max(upload, 0.0), max(download, 0.0)

    def _sample_latency(self):
        rtt_ms = None
        try:
            start = time.time()
            with socket.create_connection((self.edge_host, 80), timeout=0.2):
                rtt_ms = (time.time() - start) * 1000.0
        except Exception:
            rtt_ms = None

        if rtt_ms is not None:
            self.rtt_samples.append(rtt_ms)

        jitter_ms = 0.0
        if len(self.rtt_samples) >= 2:
            diffs = [
                abs(self.rtt_samples[index] - self.rtt_samples[index - 1])
                for index in range(1, len(self.rtt_samples))
            ]
            jitter_ms = sum(diffs) / float(len(diffs))

        return rtt_ms, jitter_ms

    def measure_bandwidth(self):
        rate = rospy.Rate(max(1, int(round(1.0 / max(self.sample_period, 0.1)))))
        while not rospy.is_shutdown():
            try:
                upload, download = self._sample_bandwidth()
                rtt_ms, jitter_ms = self._sample_latency()
                network_ok = 1.0 if rtt_ms is not None else 0.0

                msg = Float32MultiArray()
                msg.data = [
                    round(upload, 2),
                    round(download, 2),
                    round(rtt_ms, 2) if rtt_ms is not None else -1.0,
                    round(jitter_ms, 2),
                    network_ok,
                ]
                self.pub.publish(msg)

                rospy.loginfo_throttle(
                    2.0,
                    "[Bandwidth] up=%.2fMbps down=%.2fMbps rtt=%s jitter=%.2f network_ok=%s"
                    % (
                        upload,
                        download,
                        "NA" if rtt_ms is None else "%.2fms" % rtt_ms,
                        jitter_ms,
                        bool(network_ok),
                    ),
                )
            except Exception as exc:
                rospy.logwarn(f"Passive network sampling error: {exc}")

            rate.sleep()


if __name__ == "__main__":
    BandwidthMonitorNode()
